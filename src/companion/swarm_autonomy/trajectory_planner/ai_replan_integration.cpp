#include "ai_replan_integration.hpp"
#include <chrono>
#include <algorithm>
#include <iostream>
#include <sstream>

namespace swarm::autonomy {

class AIReplanIntegration::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    struct TriggerQueueItem {
        ReplanTrigger trigger;
        bool processed = false;
        ReplanResult result;
    };
    
    std::deque<TriggerQueueItem> trigger_queue_;
    std::mutex queue_mutex_;
    std::optional<ReplanResult> latest_result_;
    bool replanning_ = false;
    bool emergency_stop_ = false;
    Eigen::Vector3f emergency_hover_pos_;
    
    bool initialize(const AIReplanConfig& config, 
                    HybridPlanner* planner,
                    FormationGeometryManager* formation_mgr) {
        config_ = config;
        planner_ = planner;
        formation_mgr_ = formation_mgr;
        return planner_ != nullptr && formation_mgr_ != nullptr;
    }
    
    void processInference(const AIInferenceResult& inference,
                          const Eigen::Vector3f& position_enu,
                          const Eigen::Vector3f& velocity_enu,
                          const Eigen::Vector3f& target_enu,
                          const std::string& formation_role,
                          uint16_t leader_id) {
        
        ReplanTrigger trigger;
        trigger.timestamp_us = inference.timestamp_us;
        trigger.inference = inference;
        trigger.current_position_enu = position_enu;
        trigger.current_velocity_enu = velocity_enu;
        trigger.target_position_enu = target_enu;
        trigger.formation_role = formation_role;
        trigger.leader_id = leader_id;
        
        bool should_trigger = false;
        
        switch (inference.type) {
            case InferenceType::OBSTACLE: {
                if (inference.metadata.obstacle.threat_level >= ThreatLevel::HIGH) {
                    trigger.type = ReplanTriggerType::OBSTACLE_AVOIDANCE;
                    trigger.priority = (inference.metadata.obstacle.threat_level == ThreatLevel::CRITICAL) 
                        ? ReplanPriority::EMERGENCY : ReplanPriority::HIGH;
                    trigger.avoidance_vector = inference.metadata.obstacle.relative_velocity * -1.0f;
                    trigger.avoidance_vector.normalize();
                    trigger.avoidance_vector *= config_.min_obstacle_distance_m;
                    should_trigger = true;
                    
                    // Emergency stop check
                    if (config_.enable_emergency_stop && 
                        inference.metadata.obstacle.time_to_collision_s > 0 &&
                        inference.metadata.obstacle.time_to_collision_s < 2.0f) {
                        trigger.priority = ReplanPriority::EMERGENCY;
                        emergency_stop_ = true;
                        emergency_hover_pos_ = position_enu;
                    }
                }
                break;
            }
            case InferenceType::LANDING_SITE: {
                if (inference.metadata.landing.surface_type != SurfaceType::WATER &&
                    inference.metadata.landing.firmness_score > 0.3f) {
                    trigger.type = ReplanTriggerType::LANDING_ADJUSTMENT;
                    trigger.priority = ReplanPriority::NORMAL;
                    trigger.landing_offset_enu = Eigen::Vector3f(0, 0, -inference.metadata.landing.clearance_radius_m);
                    should_trigger = true;
                }
                break;
            }
            case InferenceType::FAULT: {
                if (inference.metadata.fault.severity >= FaultSeverity::CRITICAL) {
                    trigger.type = ReplanTriggerType::FAULT_RESPONSE;
                    trigger.priority = (inference.metadata.fault.severity == FaultSeverity::EMERGENCY)
                        ? ReplanPriority::EMERGENCY : ReplanPriority::HIGH;
                    trigger.requires_formation_sync = true;
                    trigger.affected_drones = {leader_id};  // Would be filled with formation members
                    should_trigger = true;
                }
                break;
            }
            case InferenceType::ANOMALY: {
                trigger.type = ReplanTriggerType::ANOMALY_RESPONSE;
                trigger.priority = ReplanPriority::NORMAL;
                should_trigger = true;
                break;
            }
        }
        
        if (should_trigger) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            trigger_queue_.push_back({trigger, false, {}});
        }
    }
    
    void processFormationState(const FormationState& formation_state, uint16_t drone_id) {
        // Check for formation rejoin needed
        if (formation_state.current_state != FormationState::State::HOLDING) {
            // Look for followers that need rejoin
            for (const auto& err : formation_state.position_errors_flu) {
                float lateral_err = std::sqrt(err.x() * err.x() + err.y() * err.y());
                if (lateral_err > 2.0f) {  // > 2m lateral error
                    ReplanTrigger trigger;
                    trigger.timestamp_us = formation_state.timestamp_us;
                    trigger.type = ReplanTriggerType::FORMATION_REJOIN;
                    trigger.priority = ReplanPriority::NORMAL;
                    trigger.current_position_enu = formation_state.leader_position_enu;
                    trigger.target_position_enu = formation_state.leader_position_enu;
                    trigger.formation_role = "follower";
                    trigger.formation_offset_enu = Eigen::Vector3f(err.x(), err.y(), err.z());
                    trigger.requires_formation_sync = true;
                    
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    trigger_queue_.push_back({trigger, false, {}});
                    break;
                }
            }
        }
    }
    
    std::optional<ReplanResult> processTriggers() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (trigger_queue_.empty()) return std::nullopt;
        
        // Find highest priority trigger
        auto best_it = trigger_queue_.end();
        ReplanPriority best_priority = ReplanPriority::LOW;
        
        for (auto it = trigger_queue_.begin(); it != trigger_queue_.end(); ++it) {
            if (!it->processed && it->trigger.priority > best_priority) {
                best_priority = it->trigger.priority;
                best_it = it;
            }
        }
        
        if (best_it == trigger_queue_.end()) return std::nullopt;
        
        // Process the trigger
        replanning_ = true;
        ReplanResult result = executeReplan(best_it->trigger);
        best_it->processed = true;
        best_it->result = result;
        latest_result_ = result;
        replanning_ = false;
        
        // Update stats
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.total_triggers++;
            if (result.success) {
                stats_.successful_replans++;
            } else {
                stats_.failed_replans++;
            }
            stats_.triggers_by_type[result.trigger_type]++;
            stats_.avg_replan_latency_ms = 0.9f * stats_.avg_replan_latency_ms + 0.1f * result.total_latency_ms;
            stats_.max_replan_latency_ms = std::max(stats_.max_replan_latency_ms, result.total_latency_ms);
        }
        
        // Remove processed triggers older than 1 second
        auto now = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        trigger_queue_.erase(
            std::remove_if(trigger_queue_.begin(), trigger_queue_.end(),
                [now](const TriggerQueueItem& item) {
                    return item.processed && (now - item.trigger.timestamp_us) > 1000000;
                }),
            trigger_queue_.end()
        );
        
        return result;
    }
    
    ReplanResult executeReplan(const ReplanTrigger& trigger) {
        auto start = std::chrono::high_resolution_clock::now();
        ReplanResult result;
        result.timestamp_us = trigger.timestamp_us;
        result.trigger_type = trigger.type;
        result.trigger_reason = triggerToString(trigger.type) + ": " + trigger.inference.class_name;
        
        try {
            switch (trigger.type) {
                case ReplanTriggerType::OBSTACLE_AVOIDANCE: {
                    result = replanObstacleAvoidance(trigger);
                    break;
                }
                case ReplanTriggerType::LANDING_ADJUSTMENT: {
                    result = replanLandingAdjustment(trigger);
                    break;
                }
                case ReplanTriggerType::FAULT_RESPONSE: {
                    result = replanFaultResponse(trigger);
                    break;
                }
                case ReplanTriggerType::FORMATION_REJOIN: {
                    result = replanFormationRejoin(trigger);
                    break;
                }
                case ReplanTriggerType::ANOMALY_RESPONSE: {
                    result = replanAnomalyResponse(trigger);
                    break;
                }
                default: {
                    result.success = false;
                    result.fallback_reason = "Unknown trigger type";
                }
            }
        } catch (const std::exception& e) {
            result.success = false;
            result.fallback_reason = e.what();
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.total_latency_ms = std::chrono::duration<float, std::milli>(end - start).count();
        
        // Check latency requirement
        if (result.total_latency_ms > config_.max_replan_latency_ms) {
            std::cout << "WARNING: Replan latency " << result.total_latency_ms 
                      << "ms exceeds " << config_.max_replan_latency_ms << "ms limit" << std::endl;
        }
        
        // Emergency stop handling
        if (emergency_stop_) {
            result.emergency_stop = true;
            result.emergency_hover_position_enu = emergency_hover_pos_;
            if (emergency_callback_) {
                emergency_callback_(emergency_hover_pos_, "Obstacle collision imminent");
            }
            emergency_stop_ = false;
        }
        
        // Formation sync
        if (trigger.requires_formation_sync && config_.coordinate_formation) {
            result.formation_sync_required = true;
            // Would coordinate with followers
        }
        
        // Callback
        if (replan_callback_) {
            replan_callback_(result);
        }
        
        return result;
    }
    
    ReplanResult replanObstacleAvoidance(const ReplanTrigger& trigger) {
        ReplanResult result;
        
        // Compute avoidance waypoint
        Eigen::Vector3f current = trigger.current_position_enu;
        Eigen::Vector3f avoidance_dir = trigger.avoidance_vector;
        if (avoidance_dir.norm() < 1e-3) {
            // Compute from threat position
            Eigen::Vector3f threat_pos = trigger.inference.position_enu;
            avoidance_dir = current - threat_pos;
            avoidance_dir.normalize();
        }
        
        Eigen::Vector3f avoidance_wp = current + avoidance_dir * config_.min_obstacle_distance_m;
        
        // Request replan from hybrid planner
        std::vector<Eigen::Vector3f> waypoints = {avoidance_wp, trigger.target_position_enu};
        
        auto planner_result = planner_->replanGlobal(current, trigger.current_velocity_enu, 
                                                      waypoints, 5.0f);  // 5s horizon
        
        if (planner_result.success) {
            result.success = true;
            result.new_waypoints_enu = planner_result.waypoints;
            result.waypoint_timestamps_s = planner_result.timestamps;
            result.trajectory_duration_s = planner_result.duration;
            result.new_formation_offset_enu = trigger.formation_offset_enu;
        } else {
            // Conservative fallback
            if (config_.use_conservative_fallback) {
                result = conservativeObstacleAvoidance(trigger);
            }
        }
        
        return result;
    }
    
    ReplanResult replanLandingAdjustment(const ReplanTrigger& trigger) {
        ReplanResult result;
        
        // Adjust landing approach
        Eigen::Vector3f current = trigger.current_position_enu;
        Eigen::Vector3f landing_pos = trigger.inference.position_enu;
        landing_pos += trigger.landing_offset_enu;
        
        // Replan to landing site
        std::vector<Eigen::Vector3f> waypoints = {landing_pos};
        
        auto planner_result = planner_->replanGlobal(current, trigger.current_velocity_enu,
                                                      waypoints, 10.0f);
        
        if (planner_result.success) {
            result.success = true;
            result.new_waypoints_enu = planner_result.waypoints;
            result.waypoint_timestamps_s = planner_result.timestamps;
            result.trajectory_duration_s = planner_result.duration;
        }
        
        return result;
    }
    
    ReplanResult replanFaultResponse(const ReplanTrigger& trigger) {
        ReplanResult result;
        
        switch (trigger.inference.metadata.fault.fault_type) {
            case FaultType::MOTOR_DEGRADE:
            case FaultType::BATTERY_IMBALANCE: {
                // RTL or emergency land
                if (trigger.inference.metadata.fault.recommended_action == RecommendedAction::RTL) {
                    Eigen::Vector3f home = Eigen::Vector3f(0, 0, 50);  // Home position
                    std::vector<Eigen::Vector3f> waypoints = {home};
                    
                    auto planner_result = planner_->replanGlobal(
                        trigger.current_position_enu, trigger.current_velocity_enu, waypoints, 30.0f);
                    
                    if (planner_result.success) {
                        result.success = true;
                        result.new_waypoints_enu = planner_result.waypoints;
                        result.waypoint_timestamps_s = planner_result.timestamps;
                        result.trajectory_duration_s = planner_result.duration;
                    }
                } else if (trigger.inference.metadata.fault.recommended_action == RecommendedAction::EMERGENCY_LAND) {
                    // Find nearest safe landing spot
                    result.emergency_stop = true;
                    result.emergency_hover_position_enu = trigger.current_position_enu;
                }
                break;
            }
            case FaultType::GPS_SPOOF:
            case FaultType::IMU_DRIFT:
            case FaultType::SENSOR_FAULT: {
                // Switch to VIO/UWB, hold position or RTL
                result.success = true;
                result.new_waypoints_enu = {trigger.current_position_enu};  // Hold
                result.waypoint_timestamps_s = {0.0f};
                result.trajectory_duration_s = 0.0f;
                break;
            }
            case FaultType::COMM_DEGRADE: {
                // Continue mission, store and forward
                result.success = true;
                result.new_waypoints_enu = {trigger.target_position_enu};
                break;
            }
        }
        
        return result;
    }
    
    ReplanResult replanFormationRejoin(const ReplanTrigger& trigger) {
        ReplanResult result;
        
        // Compute rejoin trajectory
        Eigen::Vector3f current = trigger.current_position_enu;
        Eigen::Vector3f target = trigger.current_position_enu + trigger.formation_offset_enu;
        
        std::vector<Eigen::Vector3f> waypoints = {target};
        
        auto planner_result = planner_->replanGlobal(current, trigger.current_velocity_enu,
                                                      waypoints, 5.0f);
        
        if (planner_result.success) {
            result.success = true;
            result.new_waypoints_enu = planner_result.waypoints;
            result.waypoint_timestamps_s = planner_result.timestamps;
            result.trajectory_duration_s = planner_result.duration;
            result.new_formation_offset_enu = trigger.formation_offset_enu;
        }
        
        return result;
    }
    
    ReplanResult replanAnomalyResponse(const ReplanTrigger& trigger) {
        ReplanResult result;
        // Conservative: hold position and assess
        result.success = true;
        result.new_waypoints_enu = {trigger.current_position_enu};
        result.waypoint_timestamps_s = {0.0f};
        result.trajectory_duration_s = 0.0f;
        return result;
    }
    
    ReplanResult conservativeObstacleAvoidance(const ReplanTrigger& trigger) {
        ReplanResult result;
        
        // Simple geometric avoidance: move perpendicular to threat
        Eigen::Vector3f current = trigger.current_position_enu;
        Eigen::Vector3f threat = trigger.inference.position_enu;
        Eigen::Vector3f threat_vel = trigger.inference.metadata.obstacle.relative_velocity;
        
        // Avoidance direction: perpendicular to threat vector in horizontal plane
        Eigen::Vector3f to_threat = threat - current;
        to_threat.z() = 0;
        if (to_threat.norm() < 1e-3) to_threat = Eigen::Vector3f(1, 0, 0);
        to_threat.normalize();
        
        Eigen::Vector3f avoid_dir = Eigen::Vector3f(-to_threat.y(), to_threat.x(), 0);
        if (threat_vel.norm() > 0.1f) {
            // Consider threat velocity
            Eigen::Vector3f threat_vel_h = threat_vel;
            threat_vel_h.z() = 0;
            if (threat_vel_h.norm() > 0) {
                float dot = avoid_dir.dot(threat_vel_h);
                if (dot < 0) avoid_dir = -avoid_dir;  // Away from threat motion
            }
        }
        
        Eigen::Vector3f safe_wp = current + avoid_dir * config_.conservative_margin_m;
        safe_wp.z() = std::max(safe_wp.z(), 10.0f);  // Minimum altitude
        
        result.success = true;
        result.new_waypoints_enu = {safe_wp};
        result.waypoint_timestamps_s = {0.0f};
        result.trajectory_duration_s = config_.conservative_margin_m / 5.0f;  // 5 m/s
        result.fallback_reason = "Conservative geometric avoidance";
        
        return result;
    }
    
    void triggerEmergencyStop(const Eigen::Vector3f& position, const std::string& reason) {
        emergency_stop_ = true;
        emergency_hover_pos_ = position;
        if (emergency_callback_) {
            emergency_callback_(position, reason);
        }
    }
    
    std::string triggerToString(ReplanTriggerType type) {
        switch (type) {
            case ReplanTriggerType::OBSTACLE_AVOIDANCE: return "Obstacle";
            case ReplanTriggerType::LANDING_ADJUSTMENT: return "Landing";
            case ReplanTriggerType::FAULT_RESPONSE: return "Fault";
            case ReplanTriggerType::ANOMALY_RESPONSE: return "Anomaly";
            case ReplanTriggerType::FORMATION_REJOIN: return "Formation";
            case ReplanTriggerType::MISSION_UPDATE: return "Mission";
        }
        return "Unknown";
    }
    
    AIReplanConfig config_;
    HybridPlanner* planner_ = nullptr;
    FormationGeometryManager* formation_mgr_ = nullptr;
    Stats stats_;
    std::mutex stats_mutex_;
    
    std::function<void(const ReplanResult&)> replan_callback_;
    std::function<void(const Eigen::Vector3f&, const std::string&)> emergency_callback_;
};

AIReplanIntegration::AIReplanIntegration() : pimpl_(std::make_unique<Impl>()) {}
AIReplanIntegration::~AIReplanIntegration() = default;
AIReplanIntegration::AIReplanIntegration(AIReplanIntegration&&) noexcept = default;
AIReplanIntegration& AIReplanIntegration::operator=(AIReplanIntegration&&) noexcept = default;

bool AIReplanIntegration::initialize(const AIReplanConfig& config, 
                                     HybridPlanner* planner,
                                     FormationGeometryManager* formation_mgr) {
    config_ = config;
    return pimpl_->initialize(config, planner, formation_mgr);
}

void AIReplanIntegration::processInference(const AIInferenceResult& inference,
                                           const Eigen::Vector3f& position_enu,
                                           const Eigen::Vector3f& velocity_enu,
                                           const Eigen::Vector3f& target_enu,
                                           const std::string& formation_role,
                                           uint16_t leader_id) {
    pimpl_->processInference(inference, position_enu, velocity_enu, target_enu, 
                             formation_role, leader_id);
}

void AIReplanIntegration::processFormationState(const FormationState& formation_state,
                                                 uint16_t drone_id) {
    pimpl_->processFormationState(formation_state, drone_id);
}

std::optional<AIReplanIntegration::ReplanResult> AIReplanIntegration::getReplanResult() {
    return pimpl_->processTriggers();
}

bool AIReplanIntegration::isReplanning() const {
    return pimpl_->replanning_;
}

void AIReplanIntegration::triggerEmergencyStop(const Eigen::Vector3f& position_enu,
                                                const std::string& reason) {
    pimpl_->triggerEmergencyStop(position_enu, reason);
}

void AIReplanIntegration::clearTriggers() {
    std::lock_guard<std::mutex> lock(pimpl_->queue_mutex_);
    pimpl_->trigger_queue_.clear();
}

const AIReplanIntegration::Stats& AIReplanIntegration::getStats() const {
    return pimpl_->stats_;
}

void AIReplanIntegration::setReplanCallback(ReplanCallback cb) {
    replan_callback_ = cb;
    pimpl_->replan_callback_ = cb;
}

void AIReplanIntegration::setEmergencyCallback(EmergencyCallback cb) {
    emergency_callback_ = cb;
    pimpl_->emergency_callback_ = cb;
}

}  // namespace swarm::autonomy