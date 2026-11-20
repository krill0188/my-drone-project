#include "heterogeneous_bidding.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace swarm::autonomy {

HeterogeneousBidder::HeterogeneousBidder(const Weights& weights) : _weights(weights) {}

HeterogeneousBidder::BidEvaluation HeterogeneousBidder::evaluateBid(
    const DroneProfile& drone,
    const Task& task,
    const AgentState& state,
    const SwarmMap* map,
    const std::vector<DroneProfile>* all_drones) {
    
    BidEvaluation eval;
    
    // Check basic feasibility
    if (!drone.capabilities.hasCapability(task.required_capabilities)) {
        eval.feasible = false;
        eval.infeasible_reason = "Missing required capabilities";
        return eval;
    }
    
    if (state.battery_pct < 0.2f) {
        eval.feasible = false;
        eval.infeasible_reason = "Insufficient battery (< 20%)";
        return eval;
    }
    
    // Check payload
    if (task.required_payload_kg > 0 && !canCarryPayload(drone.capabilities, task.required_payload_kg)) {
        eval.feasible = false;
        eval.infeasible_reason = "Insufficient payload capacity";
        return eval;
    }
    
    // Check flight time
    float estimated_duration = estimateTaskDuration(task, drone.capabilities);
    if (state.remaining_time_s < estimated_duration * 1.2f) {
        eval.feasible = false;
        eval.infeasible_reason = "Insufficient flight time";
        return eval;
    }
    
    // Compute individual costs
    eval.distance_cost = computeDistanceCost(drone, state, task);
    eval.capability_cost = computeCapabilityCost(drone, task);
    eval.energy_cost = computeEnergyCost(drone, state, task, map);
    eval.time_cost = computeTimeCost(drone, state, task);
    eval.risk_cost = computeRiskCost(drone, state, task, map);
    eval.heterogeneity_bonus = computeHeterogeneityBonus(drone, task, all_drones);
    
    // Apply platform-specific adjustments
    float base_cost = _weights.base_cost_weight * 0.0f;  // Base cost is sum of components
    
    float total = _weights.distance_weight * eval.distance_cost +
                  _weights.capability_match_weight * eval.capability_cost +
                  _weights.energy_weight * eval.energy_cost +
                  _weights.time_weight * eval.time_cost +
                  _weights.risk_weight * eval.risk_cost;
    
    // Apply heterogeneity bonus (negative cost = bonus)
    total -= _weights.heterogeneity_bonus_weight * eval.heterogeneity_bonus;
    
    // Apply platform adjustment
    total = applyPlatformAdjustment(drone, total, task.type);
    
    eval.total_cost = total;
    eval.feasible = true;
    
    // Cost breakdown
    eval.cost_breakdown = {
        {"distance", eval.distance_cost},
        {"capability", eval.capability_cost},
        {"energy", eval.energy_cost},
        {"time", eval.time_cost},
        {"risk", eval.risk_cost},
        {"heterogeneity_bonus", -eval.heterogeneity_bonus},
        {"platform_adjustment", applyPlatformAdjustment(drone, 0.0f, task.type)}
    };
    
    return eval;
}

HeterogeneousBidder::ComparisonResult HeterogeneousBidder::compareBids(
    const Task& task,
    const std::vector<DroneProfile>& drones,
    const std::vector<AgentState>& states,
    const SwarmMap* map) {
    
    ComparisonResult result;
    result.winning_cost = std::numeric_limits<float>::infinity();
    
    for (size_t i = 0; i < drones.size() && i < states.size(); ++i) {
        const auto& drone = drones[i];
        const auto& state = states[i];
        
        BidEvaluation eval = evaluateBid(drone, task, state, map, &drones);
        
        if (eval.feasible) {
            result.all_bids.emplace_back(drone.drone_id, eval.total_cost);
            
            if (eval.total_cost < result.winning_cost - 0.01f) {
                result.winning_cost = eval.total_cost;
                result.winner_id = drone.drone_id;
                result.tie = false;
            } else if (std::abs(eval.total_cost - result.winning_cost) < 0.01f) {
                // Tie: lower drone ID wins
                result.tie = true;
                if (drone.drone_id < result.winner_id) {
                    result.winner_id = drone.drone_id;
                }
            }
        }
    }
    
    if (result.all_bids.empty()) {
        result.winner_id = 0;
        result.winning_cost = std::numeric_limits<float>::infinity();
    }
    
    // Sort bids by cost
    std::sort(result.all_bids.begin(), result.all_bids.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });
    
    return result;
}

void HeterogeneousBidder::setMissionWeights(MissionType mission_type) {
    switch (mission_type) {
        case MissionType::SEARCH:
            _weights = Weights{
                1.0f, 2.0f, 1.5f, 1.5f, 1.0f, 1.5f, 0.5f
            };
            break;
        case MissionType::INSPECT:
            _weights = Weights{
                1.0f, 3.0f, 1.0f, 1.0f, 1.0f, 2.0f, 0.3f
            };
            break;
        case MissionType::PATROL:
            _weights = Weights{
                1.0f, 1.5f, 1.5f, 1.5f, 2.0f, 1.0f, 0.3f
            };
            break;
        case MissionType::ESCORT:
            _weights = Weights{
                1.0f, 2.5f, 1.0f, 1.0f, 1.5f, 2.0f, 0.3f
            };
            break;
        default:
            _weights = Weights{};
    }
}

float HeterogeneousBidder::getHoverEfficiency(const std::string& platform_type) {
    if (platform_type == "quad") return 1.0f;
    if (platform_type == "hex") return 0.9f;
    if (platform_type == "vtol") return 0.7f;
    if (platform_type == "fixed_wing") return 0.0f;  // Can't hover
    return 1.0f;
}

float HeterogeneousBidder::getForwardEfficiency(const std::string& platform_type) {
    if (platform_type == "quad") return 1.0f;
    if (platform_type == "hex") return 1.1f;
    if (platform_type == "vtol") return 1.5f;
    if (platform_type == "fixed_wing") return 2.0f;
    return 1.0f;
}

float HeterogeneousBidder::getMaxSpeed(const std::string& platform_type) {
    if (platform_type == "quad") return 15.0f;
    if (platform_type == "hex") return 18.0f;
    if (platform_type == "vtol") return 25.0f;
    if (platform_type == "fixed_wing") return 35.0f;
    return 15.0f;
}

float HeterogeneousBidder::getEnduranceFactor(const std::string& platform_type) {
    if (platform_type == "quad") return 1.0f;
    if (platform_type == "hex") return 1.2f;
    if (platform_type == "vtol") return 1.5f;
    if (platform_type == "fixed_wing") return 3.0f;
    return 1.0f;
}

bool HeterogeneousBidder::canCarryPayload(const CapabilityBitmap& caps, float payload_kg) {
    return caps.max_payload_kg >= payload_kg;
}

float HeterogeneousBidder::sensorSuitability(const CapabilityBitmap& caps, TaskType task_type) {
    switch (task_type) {
        case TaskType::COVER_AREA:
            if (caps.has_lidar_3d) return 1.0f;
            if (caps.has_stereo_camera) return 0.9f;
            if (caps.has_lidar_2d) return 0.7f;
            return 0.5f;
        case TaskType::INSPECT_POI:
            if (caps.has_stereo_camera && caps.has_ai_compute) return 1.0f;
            if (caps.has_stereo_camera) return 0.8f;
            return 0.4f;
        case TaskType::INSPECT_POI:
        case TaskType::RELAY_COMMS:
            if (caps.has_5g_modem) return 1.0f;
            if (caps.has_uwb) return 0.8f;
            return 0.5f;
        default:
            return 0.5f;
    }
}

float HeterogeneousBidder::computeDistanceCost(const DroneProfile& drone,
                                                const AgentState& state,
                                                const Task& task) {
    Vector3 task_pos;
    
    switch (task.type) {
        case TaskType::COVER_AREA:
            if (task.geometry.area) {
                // Use centroid of search area
                task_pos = task.geometry.area->centroid();
            }
            break;
        case TaskType::INSPECT_POI:
            if (task.geometry.poi_position) {
                task_pos = *task.geometry.poi_position;
            }
            break;
        case TaskType::HOLD_POSITION:
            if (task.geometry.hold_position) {
                task_pos = *task.geometry.hold_position;
            }
            break;
        case TaskType::RELAY_COMMS:
            if (task.geometry.relay_position) {
                task_pos = *task.geometry.relay_position;
            }
            break;
        case TaskType::SCOUT:
        case TaskType::ESCORT:
            if (!task.geometry.waypoints.empty()) {
                task_pos = task.geometry.waypoints.front();
            }
            break;
    }
    
    float dx = state.position.x - task_pos.x;
    float dy = state.position.y - task_pos.y;
    float dz = state.position.z - task_pos.z;
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Normalize by max expected distance (10km)
    return normalize(distance, 0.0f, 10000.0f);
}

float HeterogeneousBidder::computeCapabilityCost(const DroneProfile& drone, const Task& task) {
    const auto& required = task.required_capabilities;
    
    int missing_count = 0;
    int total_required = 0;
    
    if (required.has_rtk_gps) { total_required++; if (!drone.capabilities.has_rtk_gps) missing_count++; }
    if (required.has_stereo_camera) { total_required++; if (!drone.capabilities.has_stereo_camera) missing_count++; }
    if (required.has_lidar_2d) { total_required++; if (!drone.capabilities.has_lidar_2d) missing_count++; }
    if (required.has_lidar_3d) { total_required++; if (!drone.capabilities.has_lidar_3d) missing_count++; }
    if (required.has_uwb) { total_required++; if (!drone.capabilities.has_uwb) missing_count++; }
    if (required.has_5g_modem) { total_required++; if (!drone.capabilities.has_5g_modem) missing_count++; }
    if (required.has_ai_compute) { total_required++; if (!drone.capabilities.has_ai_compute) missing_count++; }
    
    if (total_required == 0) return 0.0f;
    return static_cast<float>(missing_count) / total_required;
}

float HeterogeneousBidder::computeEnergyCost(const DroneProfile& drone,
                                             const AgentState& state,
                                             const Task& task,
                                             const SwarmMap* map) {
    // Estimate energy required
    float task_energy = estimateTaskEnergy(task, drone.capabilities);
    
    // Available energy (normalized by battery)
    float available_energy = state.battery_pct;
    
    // Energy cost: ratio of required to available
    float required_pct = task_energy / (state.battery_voltage_v * state.remaining_time_s / 3600.0f * 1000.0f);
    
    return normalize(required_pct, 0.0f, 1.0f);
}

float HeterogeneousBidder::computeTimeCost(const DroneProfile& drone,
                                           const AgentState& state,
                                           const Task& task) {
    float estimated_duration = estimateTaskDuration(task, drone.capabilities);
    float available_time = state.remaining_time_s;
    
    if (available_time <= 0) return 1.0f;
    
    float ratio = estimated_duration / available_time;
    return normalize(ratio, 0.0f, 1.0f);
}

float HeterogeneousBidder::computeRiskCost(const DroneProfile& drone,
                                           const AgentState& state,
                                           const Task& task,
                                           const SwarmMap* map) {
    float risk = 0.0f;
    
    // Health risk
    if (state.health >= 2) risk += 0.3f;  // DEGRADED or worse
    if (state.health >= 3) risk += 0.5f;  // CRITICAL or EMERGENCY
    
    // GPS quality risk
    if (state.gps_fix_type < 3) risk += 0.3f;
    
    // Localization uncertainty
    if (state.pos_covariance_trace > 0.25f) risk += 0.2f;
    
    // No-fly zone proximity
    if (map && task.geometry.area) {
        // Check if task area near no-fly zones
    }
    
    return normalize(risk, 0.0f, 1.0f);
}

float HeterogeneousBidder::computeHeterogeneityBonus(const DroneProfile& drone,
                                                      const Task& task,
                                                      const std::vector<DroneProfile>* all_drones) {
    if (!all_drones) return 0.0f;
    
    float bonus = 0.0f;
    
    // Bonus for unique capabilities
    if (drone.capabilities.has_ai_compute && !task.required_capabilities.has_ai_compute) {
        // Check if other drones lack AI
        bool others_have_ai = false;
        for (const auto& d : *all_drones) {
            if (d.drone_id != drone.drone_id && d.capabilities.has_ai_compute) {
                others_have_ai = true;
                break;
            }
        }
        if (!others_have_ai) bonus += 0.1f;
    }
    
    if (drone.capabilities.has_lidar_3d && !task.required_capabilities.has_lidar_3d) {
        bool others_have_lidar3d = false;
        for (const auto& d : *all_drones) {
            if (d.drone_id != drone.drone_id && d.capabilities.has_lidar_3d) {
                others_have_lidar3d = true;
                break;
            }
        }
        if (!others_have_lidar3d) bonus += 0.08f;
    }
    
    if (drone.capabilities.has_5g_modem && !task.required_capabilities.has_5g_modem) {
        bool others_have_5g = false;
        for (const auto& d : *all_drones) {
            if (d.drone_id != drone.drone_id && d.capabilities.has_5g_modem) {
                others_have_5g = true;
                break;
            }
        }
        if (!others_have_5g) bonus += 0.05f;
    }
    
    return std::min(bonus, 0.2f);  // Cap at 20% bonus
}

float HeterogeneousBidder::applyPlatformAdjustment(const DroneProfile& drone, float base_cost, TaskType task_type) {
    // Platform-specific adjustments
    float adjustment = 1.0f;
    
    // VTOL and fixed-wing are more efficient for long-range tasks
    if (drone.platform_type == "vtol" || drone.platform_type == "fixed_wing") {
        if (task_type == TaskType::SCOUT || task_type == TaskType::ESCORT ||
            task_type == TaskType::RELAY_COMMS) {
            adjustment *= 0.85f;  // 15% more efficient
        }
    }
    
    // Hexacopter more stable for inspection
    if (drone.platform_type == "hex" && task_type == TaskType::INSPECT_POI) {
        adjustment *= 0.9f;
    }
    
    // Fixed-wing can't hover
    if (drone.platform_type == "fixed_wing" && 
        (task_type == TaskType::HOLD_POSITION || task_type == TaskType::INSPECT_POI)) {
        adjustment *= 1.5f;  // Penalty
    }
    
    return base_cost * adjustment;
}

float HeterogeneousBidder::normalize(float value, float min_val, float max_val) {
    if (max_val <= min_val) return 0.0f;
    float normalized = (value - min_val) / (max_val - min_val);
    return std::clamp(normalized, 0.0f, 1.0f);
}

float HeterogeneousBidder::estimateTaskDuration(const Task& task, const CapabilityBitmap& caps) {
    float base_duration = 0.0f;
    
    switch (task.type) {
        case TaskType::COVER_AREA:
            if (task.geometry.area) {
                float area_km2 = task.geometry.area->getAreaKm2();
                base_duration = area_km2 * 300.0f;
            }
            break;
        case TaskType::INSPECT_POI:
            base_duration = 300.0f;
            break;
        case TaskType::HOLD_POSITION:
            base_duration = 600.0f;
            break;
        case TaskType::RELAY_COMMS:
            base_duration = 1800.0f;
            break;
        case TaskType::SCOUT:
        case TaskType::ESCORT:
            base_duration = task.geometry.waypoints.size() * 120.0f;
            break;
    }
    
    if (caps.has_ai_compute) base_duration *= 0.9f;
    if (caps.max_flight_time_s < 1800) base_duration *= 1.1f;
    
    return base_duration;
}

float HeterogeneousBidder::estimateTaskEnergy(const Task& task, const CapabilityBitmap& caps) {
    float base_energy = 0.0f;
    
    switch (task.type) {
        case TaskType::COVER_AREA:
            base_energy = 200.0f;
            break;
        case TaskType::INSPECT_POI:
            base_energy = 50.0f;
            break;
        case TaskType::HOLD_POSITION:
            base_energy = 100.0f;
            break;
        case TaskType::RELAY_COMMS:
            base_energy = 300.0f;
            break;
        case TaskType::SCOUT:
        case TaskType::ESCORT:
            base_energy = 150.0f;
            break;
    }
    
    if (caps.max_payload_kg > 1.0f) base_energy *= 1.2f;
    
    return base_energy;
}

} // namespace swarm::autonomy