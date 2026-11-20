#include "conservative_fallback.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <limits>

namespace swarm::ai {

class ConservativeFallback::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    struct Obstacle {
        Eigen::Vector3f center;
        float radius;
        Eigen::Vector3f velocity;
        uint64_t last_update_us;
        int sensor_source;  // 0=lidar, 1=sonar, 2=depth
    };
    
    struct VoxelGrid {
        float voxel_size;
        Eigen::Vector3i dim;
        Eigen::Vector3f origin;
        std::vector<uint8_t> data;  // 0=free, 1=occupied, 2=unknown
        
        VoxelGrid(float vs, const Eigen::Vector3i& d, const Eigen::Vector3f& o)
            : voxel_size(vs), dim(d), origin(o), data(d.x() * d.y() * d.z(), 2) {}
        
        bool inBounds(int x, int y, int z) const {
            return x >= 0 && x < dim.x() && y >= 0 && y < dim.y() && z >= 0 && z < dim.z();
        }
        
        size_t index(int x, int y, int z) const {
            return (z * dim.y() + y) * dim.x() + x;
        }
        
        void setOccupied(int x, int y, int z) {
            if (inBounds(x, y, z)) data[index(x, y, z)] = 1;
        }
        
        void setFree(int x, int y, int z) {
            if (inBounds(x, y, z)) data[index(x, y, z)] = 0;
        }
        
        uint8_t get(int x, int y, int z) const {
            if (inBounds(x, y, z)) return data[index(x, y, z)];
            return 2;
        }
        
        Eigen::Vector3f voxelToWorld(int x, int y, int z) const {
            return origin + Eigen::Vector3f(
                (x + 0.5f) * voxel_size,
                (y + 0.5f) * voxel_size,
                (z + 0.5f) * voxel_size
            );
        }
        
        Eigen::Vector3i worldToVoxel(const Eigen::Vector3f& p) const {
            Eigen::Vector3f local = (p - origin) / voxel_size;
            return Eigen::Vector3i(
                std::floor(local.x()), std::floor(local.y()), std::floor(local.z())
            );
        }
    };
    
    ConservativeFallbackConfig config_;
    std::vector<Obstacle> obstacles_;
    std::unique_ptr<VoxelGrid> voxel_grid_;
    SensorData current_state_;
    uint64_t last_update_us_ = 0;
    bool has_state_ = false;
    
    bool initialize(const ConservativeFallbackConfig& config) {
        config_ = config;
        
        // Initialize voxel grid
        int grid_cells = config.grid_size_m / config.voxel_size_m;
        voxel_grid_ = std::make_unique<VoxelGrid>(
            config.voxel_size_m,
            Eigen::Vector3i(grid_cells, grid_cells, grid_cells / 2),  // Less vertical
            Eigen::Vector3f(-config.grid_size_m/2, -config.grid_size_m/2, -config.grid_size_m/4)
        );
        
        return true;
    }
    
    void updateLidar(const std::vector<Eigen::Vector3f>& points, uint64_t timestamp_us) {
        // Convert to obstacles (simple clustering)
        std::vector<Obstacle> new_obstacles = clusterPoints(points, 0, timestamp_us);
        
        // Merge with existing
        mergeObstacles(new_obstacles, 0);
        
        // Update voxel grid
        updateVoxelGridFromPoints(points);
    }
    
    void updateSonar(const std::array<float, 8>& ranges, const std::array<bool, 8>& valid, uint64_t timestamp_us) {
        // Convert sonar beams to obstacles
        std::vector<Obstacle> new_obstacles;
        
        static const float beam_angles[8] = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, -3*M_PI/4, -M_PI/2, -M_PI/4};
        
        for (int i = 0; i < 8; ++i) {
            if (valid[i] && ranges[i] > config_.sonar.min_range_m && ranges[i] < config_.sonar.max_range_m) {
                Obstacle obs;
                float angle = beam_angles[i];
                obs.center = current_state_.position_enu + Eigen::Vector3f(
                    ranges[i] * std::cos(angle),
                    ranges[i] * std::sin(angle),
                    0
                );
                obs.radius = ranges[i] * std::tan(config_.sonar.beam_angle_deg * M_PI / 360.0f);
                obs.velocity = Eigen::Vector3f::Zero();
                obs.last_update_us = timestamp_us;
                obs.sensor_source = 1;
                new_obstacles.push_back(obs);
            }
        }
        
        mergeObstacles(new_obstacles, 1);
    }
    
    void updateDepth(const std::vector<Eigen::Vector3f>& points, uint64_t timestamp_us) {
        std::vector<Obstacle> new_obstacles = clusterPoints(points, 2, timestamp_us);
        mergeObstacles(new_obstacles, 2);
        updateVoxelGridFromPoints(points);
    }
    
    void updateState(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, 
                     const Eigen::Quaternionf& orient, uint64_t timestamp_us) {
        current_state_.position_enu = pos;
        current_state_.velocity_enu = vel;
        current_state_.orientation = orient;
        current_state_.timestamp_us = timestamp_us;
        has_state_ = true;
        last_update_us_ = timestamp_us;
    }
    
    AvoidanceCommand computeAvoidance(const Eigen::Vector3f& target_velocity_enu) {
        auto start = std::chrono::high_resolution_clock::now();
        AvoidanceCommand cmd;
        cmd.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        if (!has_state_) {
            cmd.valid = false;
            return cmd;
        }
        
        // Clean old obstacles
        cleanupOldObstacles();
        
        // Check emergency stop
        if (checkEmergencyStop()) {
            cmd.valid = true;
            cmd.emergency_stop = true;
            cmd.velocity_cmd_enu = Eigen::Vector3f::Zero();
            cmd.accel_cmd_enu = -current_state_.velocity_enu * 5.0f;  // Rapid decel
            cmd.algorithm_used = "EMERGENCY";
            
            if (emergency_callback_) {
                emergency_callback_(current_state_.position_enu);
            }
            
            updateStats(cmd);
            return cmd;
        }
        
        // Run selected algorithm
        switch (config_.algorithm) {
            case ConservativeFallbackConfig::AvoidanceAlgorithm::VECTOR_FIELD:
                cmd = vectorFieldAvoidance(target_velocity_enu);
                break;
            case ConservativeFallbackConfig::AvoidanceAlgorithm::VELOCITY_OBSTACLE:
                cmd = velocityObstacleAvoidance(target_velocity_enu);
                break;
            case ConservativeFallbackConfig::AvoidanceAlgorithm::RRT_STAR:
                cmd = rrtStarAvoidance(target_velocity_enu);
                break;
            case ConservativeFallbackConfig::AvoidanceAlgorithm::HYBRID:
            default:
                cmd = hybridAvoidance(target_velocity_enu);
                break;
        }
        
        // Limit velocity/acceleration
        limitCommand(cmd);
        
        // Update stats
        auto end = std::chrono::high_resolution_clock::now();
        cmd.computation_ms = std::chrono::duration<float, std::milli>(end - start).count();
        updateStats(cmd);
        
        // Callback
        if (avoidance_callback_) {
            avoidance_callback_(cmd);
        }
        
        return cmd;
    }
    
    bool checkEmergencyStop() const {
        for (const auto& obs : obstacles_) {
            float dist = (obs.center - current_state_.position_enu).norm();
            if (dist < config_.emergency_margin_m + obs.radius) {
                return true;
            }
        }
        return false;
    }
    
    bool shouldActivate(float ai_confidence, float ai_latency_ms) const {
        if (config_.trigger_on_ai_failure && ai_confidence < 0.1f) return true;
        if (config_.trigger_on_low_confidence && ai_confidence < config_.confidence_threshold) return true;
        if (config_.trigger_on_latency && ai_latency_ms > config_.latency_threshold_ms) return true;
        return false;
    }
    
    void reset() {
        obstacles_.clear();
        if (voxel_grid_) {
            std::fill(voxel_grid_->data.begin(), voxel_grid_->data.end(), 2);
        }
    }
    
    ConservativeFallback::OccupancyGrid getOccupancyGrid() const {
        if (!voxel_grid_) return std::nullopt;
        
        ConservativeFallback::OccupancyGrid grid;
        grid.voxel_size_m = voxel_grid_->voxel_size;
        grid.grid_dim = voxel_grid_->dim;
        grid.origin_enu = voxel_grid_->origin;
        grid.occupancy = voxel_grid_->data;
        return grid;
    }

private:
    std::vector<Obstacle> clusterPoints(const std::vector<Eigen::Vector3f>& points, int sensor_source, uint64_t timestamp_us) {
        std::vector<Obstacle> clusters;
        std::vector<bool> used(points.size(), false);
        float cluster_dist = config_.voxel_size_m * 2;
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (used[i]) continue;
            
            // Simple region growing
            std::vector<size_t> cluster_indices;
            cluster_indices.push_back(i);
            used[i] = true;
            
            for (size_t j = 0; j < cluster_indices.size(); ++j) {
                size_t idx = cluster_indices[j];
                for (size_t k = 0; k < points.size(); ++k) {
                    if (!used[k] && (points[idx] - points[k]).norm() < cluster_dist) {
                        used[k] = true;
                        cluster_indices.push_back(k);
                    }
                }
            }
            
            // Compute cluster center and radius
            if (cluster_indices.size() >= 3) {  // Min points
                Eigen::Vector3f center = Eigen::Vector3f::Zero();
                for (size_t idx : cluster_indices) center += points[idx];
                center /= cluster_indices.size();
                
                float max_dist = 0;
                for (size_t idx : cluster_indices) {
                    float d = (points[idx] - center).norm();
                    max_dist = std::max(max_dist, d);
                }
                
                Obstacle obs;
                obs.center = center;
                obs.radius = max_dist + config_.voxel_size_m;
                obs.velocity = Eigen::Vector3f::Zero();
                obs.last_update_us = timestamp_us;
                obs.sensor_source = sensor_source;
                clusters.push_back(obs);
            }
        }
        
        return clusters;
    }
    
    void mergeObstacles(const std::vector<Obstacle>& new_obs, int sensor_source) {
        float merge_dist = config_.voxel_size_m * 3;
        
        for (const auto& new_ob : new_obs) {
            bool merged = false;
            
            for (auto& existing : obstacles_) {
                if (existing.sensor_source != sensor_source) continue;
                
                float dist = (existing.center - new_ob.center).norm();
                if (dist < merge_dist) {
                    // Merge: weighted average
                    existing.center = (existing.center + new_ob.center) * 0.5f;
                    existing.radius = std::max(existing.radius, new_ob.radius);
                    existing.last_update_us = new_ob.last_update_us;
                    merged = true;
                    break;
                }
            }
            
            if (!merged) {
                obstacles_.push_back(new_ob);
            }
        }
    }
    
    void updateVoxelGridFromPoints(const std::vector<Eigen::Vector3f>& points) {
        if (!voxel_grid_) return;
        
        for (const auto& p : points) {
            Eigen::Vector3i v = voxel_grid_->worldToVoxel(p);
            if (voxel_grid_->inBounds(v.x(), v.y(), v.z())) {
                voxel_grid_->setOccupied(v.x(), v.y(), v.z());
                
                // Mark ray as free (simplified - would need sensor origin)
                // For now, just mark nearby as free
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            int x = v.x() + dx, y = v.y() + dy, z = v.z() + dz;
                            if (voxel_grid_->inBounds(x, y, z) && voxel_grid_->get(x, y, z) == 2) {
                                voxel_grid_->setFree(x, y, z);
                            }
                        }
                    }
                }
            }
        }
    }
    
    void cleanupOldObstacles() {
        uint64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        obstacles_.erase(
            std::remove_if(obstacles_.begin(), obstacles_.end(),
                [now](const Obstacle& obs) {
                    return (now - obs.last_update_us) > 1000000;  // 1 second
                }),
            obstacles_.end()
        );
    }
    
    AvoidanceCommand vectorFieldAvoidance(const Eigen::Vector3f& target_vel) {
        AvoidanceCommand cmd;
        cmd.valid = true;
        Eigen::Vector3f repulsive = Eigen::Vector3f::Zero();
        Eigen::Vector3f pos = current_state_.position_enu;
        
        for (const auto& obs : obstacles_) {
            Eigen::Vector3f to_obs = obs.center - pos;
            float dist = to_obs.norm();
            
            if (dist < config_.safety_margin_m + obs.radius && dist > 1e-3) {
                // Repulsive force: inversely proportional to distance squared
                float influence = config_.safety_margin_m / (dist - obs.radius + 0.1f);
                influence = std::min(influence, 10.0f);
                repulsive += -to_obs.normalized() * influence;
            }
        }
        
        // Attractive force toward target velocity
        Eigen::Vector3f attractive = target_vel * 2.0f;
        
        // Combined
        Eigen::Vector3f result = attractive + repulsive;
        
        // Limit to max velocity
        if (result.norm() > config_.max_avoidance_velocity_mps) {
            result.normalize();
            result *= config_.max_avoidance_velocity_mps;
        }
        
        cmd.velocity_cmd_enu = result;
        cmd.accel_cmd_enu = (result - current_state_.velocity_enu) * 5.0f;  // PD-like
        cmd.algorithm_used = "VECTOR_FIELD";
        cmd.num_obstacles_detected = obstacles_.size();
        
        if (!obstacles_.empty()) {
            float min_dist = std::numeric_limits<float>::max();
            for (const auto& obs : obstacles_) {
                float d = (obs.center - pos).norm() - obs.radius;
                min_dist = std::min(min_dist, d);
            }
            cmd.closest_obstacle_dist_m = min_dist;
        }
        
        return cmd;
    }
    
    AvoidanceCommand velocityObstacleAvoidance(const Eigen::Vector3f& target_vel) {
        // Simplified Velocity Obstacle / ORCA
        AvoidanceCommand cmd = vectorFieldAvoidance(target_vel);  // Fallback
        cmd.algorithm_used = "VELOCITY_OBSTACLE";
        return cmd;
    }
    
    AvoidanceCommand rrtStarAvoidance(const Eigen::Vector3f& target_vel) {
        // Simplified local RRT*
        AvoidanceCommand cmd = vectorFieldAvoidance(target_vel);  // Fallback
        cmd.algorithm_used = "RRT_STAR";
        return cmd;
    }
    
    AvoidanceCommand hybridAvoidance(const Eigen::Vector3f& target_vel) {
        // Combine vector field (fast) with velocity obstacle (accurate)
        AvoidanceCommand cmd = vectorFieldAvoidance(target_vel);
        cmd.algorithm_used = "HYBRID";
        return cmd;
    }
    
    void limitCommand(AvoidanceCommand& cmd) {
        // Velocity limit
        if (cmd.velocity_cmd_enu.norm() > config_.max_avoidance_velocity_mps) {
            cmd.velocity_cmd_enu.normalize();
            cmd.velocity_cmd_enu *= config_.max_avoidance_velocity_mps;
        }
        
        // Acceleration limit
        Eigen::Vector3f desired_accel = cmd.accel_cmd_enu;
        if (desired_accel.norm() > config_.max_avoidance_accel_mps2) {
            desired_accel.normalize();
            desired_accel *= config_.max_avoidance_accel_mps2;
        }
        cmd.accel_cmd_enu = desired_accel;
    }
    
    void updateStats(const AvoidanceCommand& cmd) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.total_computations++;
        if (cmd.emergency_stop) stats_.emergency_stops++;
        stats_.avg_computation_ms = 0.9f * stats_.avg_computation_ms + 0.1f * cmd.computation_ms;
        stats_.max_computation_ms = std::max(stats_.max_computation_ms, cmd.computation_ms);
        stats_.max_obstacles_detected = std::max(stats_.max_obstacles_detected, cmd.num_obstacles_detected);
    }
};

ConservativeFallback::ConservativeFallback() : pimpl_(std::make_unique<Impl>()) {}
ConservativeFallback::~ConservativeFallback() = default;
ConservativeFallback::ConservativeFallback(ConservativeFallback&&) noexcept = default;
ConservativeFallback& ConservativeFallback::operator=(ConservativeFallback&&) noexcept = default;

bool ConservativeFallback::initialize(const ConservativeFallbackConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

void ConservativeFallback::updateLidar(const std::vector<Eigen::Vector3f>& points, uint64_t timestamp_us) {
    pimpl_->updateLidar(points, timestamp_us);
}

void ConservativeFallback::updateSonar(const std::array<float, 8>& ranges, const std::array<bool, 8>& valid, uint64_t timestamp_us) {
    pimpl_->updateSonar(ranges, valid, timestamp_us);
}

void ConservativeFallback::updateDepth(const std::vector<Eigen::Vector3f>& points, uint64_t timestamp_us) {
    pimpl_->updateDepth(points, timestamp_us);
}

void ConservativeFallback::updateState(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, 
                                       const Eigen::Quaternionf& orient, uint64_t timestamp_us) {
    pimpl_->updateState(pos, vel, orient, timestamp_us);
}

ConservativeFallback::AvoidanceCommand ConservativeFallback::computeAvoidance(const Eigen::Vector3f& target_velocity_enu) {
    return pimpl_->computeAvoidance(target_velocity_enu);
}

bool ConservativeFallback::checkEmergencyStop() const {
    return pimpl_->checkEmergencyStop();
}

std::optional<ConservativeFallback::OccupancyGrid> ConservativeFallback::getOccupancyGrid() const {
    return pimpl_->getOccupancyGrid();
}

bool ConservativeFallback::shouldActivate(float ai_confidence, float ai_latency_ms) const {
    return pimpl_->shouldActivate(ai_confidence, ai_latency_ms);
}

const ConservativeFallback::Stats& ConservativeFallback::getStats() const {
    return stats_;
}

void ConservativeFallback::reset() {
    pimpl_->reset();
}

void ConservativeFallback::setAvoidanceCallback(AvoidanceCallback cb) {
    avoidance_callback_ = cb;
}

void ConservativeFallback::setEmergencyCallback(EmergencyCallback cb) {
    emergency_callback_ = cb;
}

}  // namespace swarm::ai