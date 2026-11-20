#include "capability.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace swarm::autonomy {

// ============================================================================
// CapabilityBitmap Implementation
// ============================================================================

bool CapabilityBitmap::hasCapability(const CapabilityBitmap& required) const {
    if (required.has_rtk_gps && !has_rtk_gps) return false;
    if (required.has_stereo_camera && !has_stereo_camera) return false;
    if (required.has_lidar_2d && !has_lidar_2d) return false;
    if (required.has_lidar_3d && !has_lidar_3d) return false;
    if (required.has_uwb && !has_uwb) return false;
    if (required.has_5g_modem && !has_5g_modem) return false;
    if (required.has_ai_compute && !has_ai_compute) return false;
    
    if (required.max_flight_time_s > max_flight_time_s) return false;
    if (required.max_payload_kg > max_payload_kg) return false;
    
    // Communication tier: must have at least the required tiers
    if ((required.comm_tier_bitmap & comm_tier_bitmap) != required.comm_tier_bitmap) {
        return false;
    }
    
    return true;
}

uint8_t CapabilityBitmap::getCommTierCount() const {
    uint8_t count = 0;
    if (comm_tier_bitmap & 0b001) count++;  // WiFi 6E
    if (comm_tier_bitmap & 0b010) count++;  // 5G
    if (comm_tier_bitmap & 0b100) count++;  // LoRa
    return count;
}

std::string CapabilityBitmap::toString() const {
    std::stringstream ss;
    ss << "Capabilities: ";
    bool first = true;
    
    if (has_rtk_gps) { ss << (first ? "" : ", ") << "RTK GPS"; first = false; }
    if (has_stereo_camera) { ss << (first ? "" : ", ") << "Stereo Cam"; first = false; }
    if (has_lidar_2d) { ss << (first ? "" : ", ") << "2D LiDAR"; first = false; }
    if (has_lidar_3d) { ss << (first ? "" : ", ") << "3D LiDAR"; first = false; }
    if (has_uwb) { ss << (first ? "" : ", ") << "UWB"; first = false; }
    if (has_5g_modem) { ss << (first ? "" : ", ") << "5G"; first = false; }
    if (has_ai_compute) { ss << (first ? "" : ", ") << "AI Compute"; first = false; }
    
    if (max_flight_time_s > 0) {
        ss << (first ? "" : ", ") << "Flight: " << max_flight_time_s << "s"; first = false;
    }
    if (max_payload_kg > 0) {
        ss << (first ? "" : ", ") << "Payload: " << max_payload_kg << "kg"; first = false;
    }
    
    ss << " | Comm: ";
    if (comm_tier_bitmap & 0b001) ss << "WiFi6E ";
    if (comm_tier_bitmap & 0b010) ss << "5G ";
    if (comm_tier_bitmap & 0b100) ss << "LoRa ";
    
    return ss.str();
}

std::vector<std::string> CapabilityBitmap::getCapabilitiesList() const {
    std::vector<std::string> caps;
    if (has_rtk_gps) caps.push_back("RTK_GPS");
    if (has_stereo_camera) caps.push_back("STEREO_CAM");
    if (has_lidar_2d) caps.push_back("LIDAR_2D");
    if (has_lidar_3d) caps.push_back("LIDAR_3D");
    if (has_uwb) caps.push_back("UWB");
    if (has_5g_modem) caps.push_back("5G");
    if (has_ai_compute) caps.push_back("AI_COMPUTE");
    return caps;
}

std::string CapabilityBitmap::toJson() const {
    std::stringstream ss;
    ss << "{";
    ss << "\"has_rtk_gps\":" << (has_rtk_gps ? "true" : "false") << ",";
    ss << "\"has_stereo_camera\":" << (has_stereo_camera ? "true" : "false") << ",";
    ss << "\"has_lidar_2d\":" << (has_lidar_2d ? "true" : "false") << ",";
    ss << "\"has_lidar_3d\":" << (has_lidar_3d ? "true" : "false") << ",";
    ss << "\"has_uwb\":" << (has_uwb ? "true" : "false") << ",";
    ss << "\"has_5g_modem\":" << (has_5g_modem ? "true" : "false") << ",";
    ss << "\"has_ai_compute\":" << (has_ai_compute ? "true" : "false") << ",";
    ss << "\"max_flight_time_s\":" << max_flight_time_s << ",";
    ss << "\"max_payload_kg\":" << max_payload_kg << ",";
    ss << "\"comm_tier_bitmap\":" << static_cast<int>(comm_tier_bitmap);
    ss << "}";
    return ss.str();
}

std::optional<CapabilityBitmap> CapabilityBitmap::fromJson(const std::string& json) {
    // Simplified JSON parsing - in real implementation would use nlohmann/json
    CapabilityBitmap caps;
    // Would parse JSON here
    return caps;
}

std::string CapabilityBitmap::toYaml() const {
    std::stringstream ss;
    ss << "has_rtk_gps: " << (has_rtk_gps ? "true" : "false") << "\n";
    ss << "has_stereo_camera: " << (has_stereo_camera ? "true" : "false") << "\n";
    ss << "has_lidar_2d: " << (has_lidar_2d ? "true" : "false") << "\n";
    ss << "has_lidar_3d: " << (has_lidar_3d ? "true" : "false") << "\n";
    ss << "has_uwb: " << (has_uwb ? "true" : "false") << "\n";
    ss << "has_5g_modem: " << (has_5g_modem ? "true" : "false") << "\n";
    ss << "has_ai_compute: " << (has_ai_compute ? "true" : "false") << "\n";
    ss << "max_flight_time_s: " << max_flight_time_s << "\n";
    ss << "max_payload_kg: " << max_payload_kg << "\n";
    ss << "comm_tier_bitmap: " << static_cast<int>(comm_tier_bitmap) << "\n";
    return ss.str();
}

std::optional<CapabilityBitmap> CapabilityBitmap::fromYaml(const std::string& yaml) {
    // Simplified YAML parsing
    return std::nullopt;
}

// ============================================================================
// BidScorer Implementation
// ============================================================================

BidScorer::BidScore BidScorer::computeBid(const CapabilityBitmap& drone_caps,
                                           const AgentState& drone_state,
                                           const Task& task,
                                           const Weights& weights,
                                           const SwarmMap* map) {
    BidScore score;
    
    // Check feasibility first
    if (!drone_caps.hasCapability(task.required_capabilities)) {
        score.feasible = false;
        score.infeasible_reason = "Missing required capabilities";
        return score;
    }
    
    // Check battery
    if (drone_state.battery_pct < 0.3f) {  // 30% minimum
        score.feasible = false;
        score.infeasible_reason = "Insufficient battery";
        return score;
    }
    
    // Check flight time
    float estimated_duration = estimateTaskDuration(task, drone_caps);
    if (drone_state.remaining_time_s < estimated_duration * 1.2f) {  // 20% margin
        score.feasible = false;
        score.infeasible_reason = "Insufficient flight time";
        return score;
    }
    
    // Compute individual costs
    score.distance_cost = computeDistanceCost(drone_state.position, task.geometry);
    score.capability_cost = computeCapabilityCost(drone_caps, task);
    score.energy_cost = computeEnergyCost(drone_state, task, nullptr);
    score.time_cost = computeTimeCost(drone_state, task, nullptr);
    score.risk_cost = computeRiskCost(drone_state, task, nullptr);
    
    // Apply weights
    score.total_cost = weights.distance_weight * score.distance_cost +
                       weights.capability_weight * score.capability_cost +
                       weights.energy_weight * score.energy_cost +
                       weights.time_weight * score.time_cost +
                       weights.risk_weight * score.risk_cost;
    
    // Apply heterogeneous adjustment
    score.total_cost = applyHeterogeneousAdjustment(drone_caps, task, score.total_cost);
    
    score.feasible = true;
    return score;
}

float BidScorer::computeDistanceCost(const Vector3& drone_pos, 
                                      const TaskGeometry& geometry) {
    Vector3 task_pos;
    
    switch (geometry.type) {
        case TaskType::COVER_AREA:
            if (geometry.area) {
                task_pos = geometry.area->centroid();
            }
            break;
        case TaskType::INSPECT_POI:
            if (geometry.poi_position) {
                task_pos = *geometry.poi_position;
            }
            break;
        case TaskType::HOLD_POSITION:
            if (geometry.hold_position) {
                task_pos = *geometry.hold_position;
            }
            break;
        case TaskType::RELAY_COMMS:
            if (geometry.relay_position) {
                task_pos = *geometry.relay_position;
            }
            break;
        case TaskType::SCOUT:
        case TaskType::ESCORT:
            if (!geometry.waypoints.empty()) {
                task_pos = geometry.waypoints.front();
            }
            break;
    }
    
    float dx = drone_pos.x - task_pos.x;
    float dy = drone_pos.y - task_pos.y;
    float dz = drone_pos.z - task_pos.z;
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Normalize by max expected distance (10km)
    return normalize(distance, 0.0f, 10000.0f);
}

float BidScorer::computeCapabilityCost(const CapabilityBitmap& drone_caps,
                                        const Task& task) {
    // Check required capabilities
    const CapabilityBitmap& required = task.required_capabilities;
    
    int missing_count = 0;
    int total_required = 0;
    
    if (required.has_rtk_gps) { total_required++; if (!drone_caps.has_rtk_gps) missing_count++; }
    if (required.has_stereo_camera) { total_required++; if (!drone_caps.has_stereo_camera) missing_count++; }
    if (required.has_lidar_2d) { total_required++; if (!drone_caps.has_lidar_2d) missing_count++; }
    if (required.has_lidar_3d) { total_required++; if (!drone_caps.has_lidar_3d) missing_count++; }
    if (required.has_uwb) { total_required++; if (!drone_caps.has_uwb) missing_count++; }
    if (required.has_5g_modem) { total_required++; if (!drone_caps.has_5g_modem) missing_count++; }
    if (required.has_ai_compute) { total_required++; if (!drone_caps.has_ai_compute) missing_count++; }
    
    if (total_required == 0) return 0.0f;
    
    // Cost proportional to missing capabilities
    return static_cast<float>(missing_count) / total_required;
}

float BidScorer::computeEnergyCost(const AgentState& state,
                                    const Task& task,
                                    const SwarmMap* map) {
    // Estimate energy required for task
    float task_energy = estimateEnergyConsumption(task, state.capabilities);
    
    // Available energy (normalized)
    float available_energy = state.battery_pct;  // 0.0 - 1.0
    
    // Energy cost: ratio of required to available
    float required_pct = task_energy / (state.battery_voltage_v * state.remaining_time_s / 3600.0f * 1000.0f);  // Simplified
    
    return normalize(required_pct, 0.0f, 1.0f);
}

float BidScorer::computeTimeCost(const AgentState& state,
                                  const Task& task,
                                  const SwarmMap* map) {
    float estimated_duration = estimateTaskDuration(task, state.capabilities);
    float available_time = state.remaining_time_s;
    
    if (available_time <= 0) return 1.0f;
    
    // Time cost: ratio of task duration to available time
    float ratio = estimated_duration / available_time;
    return normalize(ratio, 0.0f, 1.0f);
}

float BidScorer::computeRiskCost(const AgentState& state,
                                  const Task& task,
                                  const SwarmMap* map) {
    float risk = 0.0f;
    
    // Weather risk (from agent state)
    if (state.health != 0) risk += 0.2f;  // DEGRADED or worse
    
    // GPS quality risk
    if (state.gps_fix_type < 3) risk += 0.3f;  // Not RTK fixed
    
    // Localization uncertainty
    if (state.pos_covariance_trace > 0.25f) risk += 0.2f;  // > 0.5m 1-sigma
    
    // Communication quality
    // Would check mesh QoS here
    
    // No-fly zone proximity (if map available)
    if (map) {
        // Check if task area near no-fly zones
    }
    
    return normalize(risk, 0.0f, 1.0f);
}

float BidScorer::applyHeterogeneousAdjustment(const CapabilityBitmap& drone_caps,
                                               const Task& task,
                                               float base_cost) {
    // Boost cost for drones with better-than-required capabilities
    // (they're more valuable, so lower effective cost)
    
    int extra_caps = 0;
    if (drone_caps.has_ai_compute && !task.required_capabilities.has_ai_compute) extra_caps++;
    if (drone_caps.has_lidar_3d && !task.required_capabilities.has_lidar_3d) extra_caps++;
    if (drone_caps.has_5g_modem && !task.required_capabilities.has_5g_modem) extra_caps++;
    
    // Small bonus for extra capabilities (max 10% reduction)
    if (extra_caps > 0) {
        float reduction = std::min(0.1f, extra_caps * 0.03f);
        return base_cost * (1.0f - reduction);
    }
    
    return base_cost;
}

float BidScorer::normalize(float value, float min_val, float max_val) {
    if (max_val <= min_val) return 0.0f;
    float normalized = (value - min_val) / (max_val - min_val);
    return std::clamp(normalized, 0.0f, 1.0f);
}

float BidScorer::estimateTaskDuration(const Task& task, const CapabilityBitmap& caps) {
    float base_duration = 0.0f;
    
    switch (task.type) {
        case TaskType::COVER_AREA:
            if (task.geometry.area) {
                float area_km2 = task.geometry.area->getAreaKm2();
                base_duration = area_km2 * 300.0f;  // 5 min per km²
            }
            break;
        case TaskType::INSPECT_POI:
            base_duration = 300.0f;  // 5 min per inspection
            break;
        case TaskType::HOLD_POSITION:
            base_duration = task.geometry.hold_radius ? 600.0f : 300.0f;
            break;
        case TaskType::RELAY_COMMS:
            base_duration = 1800.0f;  // 30 min
            break;
        case TaskType::SCOUT:
        case TaskType::ESCORT:
            base_duration = task.geometry.waypoints.size() * 120.0f;  // 2 min per waypoint
            break;
    }
    
    // Adjust for capabilities
    if (caps.has_ai_compute) base_duration *= 0.9f;  // Faster with AI
    if (caps.max_flight_time_s < 1800) base_duration *= 1.1f;  // Less endurance
    
    return base_duration;
}

float BidScorer::estimateEnergyConsumption(const Task& task, const CapabilityBitmap& caps) {
    float base_energy = 0.0f;  // Wh
    
    switch (task.type) {
        case TaskType::COVER_AREA:
            base_energy = 200.0f;  // 200 Wh for typical search
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
    
    // Adjust for payload
    if (caps.max_payload_kg > 1.0f) base_energy *= 1.2f;
    
    return base_energy;
}

} // namespace swarm::autonomy