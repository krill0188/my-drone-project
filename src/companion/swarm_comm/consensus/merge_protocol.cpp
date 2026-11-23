#include "merge_protocol.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <numeric>

namespace swarm::companion {

SplitBrainMergeProtocol::SplitBrainMergeProtocol() {
    _config.partition_detection_timeout_ms = 5000;
    _config.merge_timeout_ms = 30000;
    _config.heartbeat_interval_ms = 100;
    _config.max_missed_heartbeats = 5;
    _config.auto_resolve_safety = true;
    _config.require_manual_safety_approval = false;
    _config.min_vehicles_for_quorum = 2;
    _config.enable_formation_recovery = true;
    _config.link_quality_threshold = 0.3;
}

SplitBrainMergeProtocol::~SplitBrainMergeProtocol() {
    if (_merge_thread.joinable()) {
        cancelMerge();
        _merge_thread.join();
    }
}

bool SplitBrainMergeProtocol::initialize(const MergeProtocolConfig& config) {
    std::lock_guard<std::mutex> lock(_mutex);
    _config = config;
    _state = PartitionState::NORMAL;
    _vehicles.clear();
    _last_known_vehicles.clear();
    _connectivity_matrix.clear();
    _last_heartbeat.clear();
    _missed_heartbeats.clear();
    _current_partition.reset();
    _partition_history.clear();
    return true;
}

void SplitBrainMergeProtocol::setSafetyCallback(SafetyCallback callback) {
    std::lock_guard<std::mutex> lock(_mutex);
    _safety_callback = std::move(callback);
}

void SplitBrainMergeProtocol::setMergeCallback(MergeCallback callback) {
    std::lock_guard<std::mutex> lock(_mutex);
    _merge_callback = std::move(callback);
}

void SplitBrainMergeProtocol::updateVehicleState(const VehicleStateSnapshot& state) {
    std::lock_guard<std::mutex> lock(_mutex);
    _vehicles[state.vehicle_id] = state;
    _last_known_vehicles[state.vehicle_id] = state;
    _last_heartbeat[state.vehicle_id] = state.timestamp_us;
    _missed_heartbeats[state.vehicle_id] = 0;

    // Update connectivity matrix
    if (_connectivity_matrix.find(state.vehicle_id) == _connectivity_matrix.end()) {
        _connectivity_matrix[state.vehicle_id] = {};
    }
}

void SplitBrainMergeProtocol::removeVehicle(uint8_t vehicle_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    _vehicles.erase(vehicle_id);
    _last_heartbeat.erase(vehicle_id);
    _missed_heartbeats.erase(vehicle_id);
    _connectivity_matrix.erase(vehicle_id);
    for (auto& [id, links] : _connectivity_matrix) {
        links.erase(vehicle_id);
    }
}

void SplitBrainMergeProtocol::checkPartitions() {
    std::lock_guard<std::mutex> lock(_mutex);
    
    uint64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    if (now - _last_partition_check < _config.heartbeat_interval_ms * 1000) {
        return;
    }
    _last_partition_check = now;

    checkVehicleHeartbeats();
    updateConnectivityMatrix();
    detectPartition();
}

void SplitBrainMergeProtocol::checkVehicleHeartbeats() {
    uint64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    uint64_t timeout_us = _config.heartbeat_interval_ms * _config.max_missed_heartbeats * 1000;

    for (auto& [id, last_hb] : _last_heartbeat) {
        if (now - last_hb > timeout_us) {
            _missed_heartbeats[id]++;
        } else {
            _missed_heartbeats[id] = 0;
        }
    }
}

void SplitBrainMergeProtocol::updateConnectivityMatrix() {
    // Build connectivity from vehicle states
    // In real implementation, this would come from actual link quality data
    for (const auto& [id_a, vehicle_a] : _vehicles) {
        if (_connectivity_matrix.find(vehicle_a.vehicle_id) == _connectivity_matrix.end()) {
            _connectivity_matrix[vehicle_a.vehicle_id] = {};
        }

        for (const auto& [id_b, vehicle_b] : _vehicles) {
            if (vehicle_a.vehicle_id == vehicle_b.vehicle_id) continue;

            // Simple distance-based connectivity (would be replaced with actual link quality)
            double distance = std::hypot(
                vehicle_a.latitude - vehicle_b.latitude,
                vehicle_a.longitude - vehicle_b.longitude
            );
            
            // Simplified: link quality inversely proportional to distance
            double quality = std::max(0.0, 1.0 - distance / 10000.0);  // 10km range
            _connectivity_matrix[vehicle_a.vehicle_id][vehicle_b.vehicle_id] = quality;
        }
    }
}

void SplitBrainMergeProtocol::detectPartition() {
    if (_state != PartitionState::NORMAL && _state != PartitionState::PARTITION_DETECTED) {
        return;  // Already in partition or merge
    }

    // Find connected components using BFS
    std::set<uint8_t> visited;
    std::vector<std::set<uint8_t>> components;

    for (const auto& [id, _] : _vehicles) {
        if (visited.count(id)) continue;

        std::set<uint8_t> component;
        std::queue<uint8_t> q;
        q.push(id);
        visited.insert(id);

        while (!q.empty()) {
            uint8_t current = q.front();
            q.pop();
            component.insert(current);

            // Check connectivity
            if (_connectivity_matrix.find(current) != _connectivity_matrix.end()) {
                for (const auto& [neighbor, quality] : _connectivity_matrix[current]) {
                    if (quality >= _config.link_quality_threshold && 
                        !visited.count(neighbor) &&
                        _vehicles.find(neighbor) != _vehicles.end()) {
                        visited.insert(neighbor);
                        q.push(neighbor);
                    }
                }
            }
        }

        if (!component.empty()) {
            components.push_back(component);
        }
    }

    // Check if we have multiple components (partition)
    if (components.size() > 1) {
        // Partition detected
        PartitionInfo partition;
        partition.detected_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        partition.detection_method = "connectivity_loss";

        // Split components into two partitions (largest two)
        std::sort(components.begin(), components.end(),
                  [](const auto& a, const auto& b) { return a.size() > b.size(); });

        if (components.size() >= 2) {
            partition.partition_a_vehicles = components[0];
            partition.partition_b_vehicles = components[1];
        } else {
            partition.partition_a_vehicles = components[0];
        }

        // Add remaining components to partition B
        for (size_t i = 2; i < components.size(); ++i) {
            partition.partition_b_vehicles.insert(components[i].begin(), components[i].end());
        }

        onPartitionDetected(partition);
    }
}

void SplitBrainMergeProtocol::onPartitionDetected(const PartitionInfo& partition) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (_state == PartitionState::PARTITION_DETECTED && _current_partition.has_value()) {
        // Already in partition, update existing
        _current_partition = partition;
    } else {
        _state = PartitionState::PARTITION_DETECTED;
        _current_partition = partition;
        _partition_history.push_back(partition);
        
        // Save last known states before partition
        _last_known_vehicles = _vehicles;
        
        std::cout << "[MergeProtocol] Partition detected: "
                  << partition.partition_a_vehicles.size() << " vs "
                  << partition.partition_b_vehicles.size() << " vehicles" << std::endl;

        // Start merge process automatically
        if (_config.auto_resolve_safety) {
            startMerge(partition);
        }
    }
}

bool SplitBrainMergeProtocol::startMerge(const PartitionInfo& partition) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    if (_merge_in_progress.load()) {
        return false;  // Merge already in progress
    }

    if (_state != PartitionState::PARTITION_DETECTED) {
        return false;  // No partition to merge
    }

    _state = PartitionState::MERGE_IN_PROGRESS;
    _merge_in_progress.store(true);

    // Start merge in background thread
    _merge_thread = std::thread([this, partition]() {
        MergeResult result = performMerge(partition);
        finalizeMerge(result);
    });

    return true;
}

MergeResult SplitBrainMergeProtocol::performMerge(const PartitionInfo& partition) {
    MergeResult result;
    result.merge_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    auto start_time = std::chrono::steady_clock::now();

    // Build vehicle maps for both partitions
    std::map<uint8_t, VehicleStateSnapshot> partition_a, partition_b;
    buildVehicleMaps(partition, partition_a, partition_b);

    // Detect conflicts
    result.conflicts = detectConflicts(partition_a, partition_b);

    // Resolve conflicts
    resolveAllConflicts(result.conflicts);

    // Merge vehicle states
    for (const auto& [id, vehicle_a] : partition_a) {
        auto it_b = partition_b.find(id);
        if (it_b != partition_b.end()) {
            // Vehicle in both partitions - merge
            VehicleStateSnapshot merged;
            mergeVehicleStates(vehicle_a, it_b->second, merged);
            result.merged_vehicles[id] = merged;
        } else {
            // Vehicle only in partition A
            result.merged_vehicles[id] = vehicle_a;
        }
    }

    // Add vehicles only in partition B
    for (const auto& [id, vehicle_b] : partition_b) {
        if (partition_a.find(id) == partition_a.end()) {
            result.merged_vehicles[id] = vehicle_b;
        }
    }

    result.vehicles_merged = result.merged_vehicles.size();
    result.success = true;
    result.merge_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start_time).count();

    return result;
}

void SplitBrainMergeProtocol::buildVehicleMaps(const PartitionInfo& partition,
                                                std::map<uint8_t, VehicleStateSnapshot>& partition_a,
                                                std::map<uint8_t, VehicleStateSnapshot>& partition_b) {
    std::lock_guard<std::mutex> lock(_mutex);

    for (uint8_t id : partition.partition_a_vehicles) {
        if (_last_known_vehicles.find(id) != _last_known_vehicles.end()) {
            partition_a[id] = _last_known_vehicles[id];
        } else if (_vehicles.find(id) != _vehicles.end()) {
            partition_a[id] = _vehicles[id];
        }
    }

    for (uint8_t id : partition.partition_b_vehicles) {
        if (_last_known_vehicles.find(id) != _last_known_vehicles.end()) {
            partition_b[id] = _last_known_vehicles[id];
        } else if (_vehicles.find(id) != _vehicles.end()) {
            partition_b[id] = _vehicles[id];
        }
    }
}

std::vector<MergeConflict> SplitBrainMergeProtocol::detectConflicts(
    const std::map<uint8_t, VehicleStateSnapshot>& partition_a,
    const std::map<uint8_t, VehicleStateSnapshot>& partition_b) {

    std::vector<MergeConflict> conflicts;

    for (const auto& [id, vehicle_a] : partition_a) {
        auto it_b = partition_b.find(id);
        if (it_b == partition_b.end()) continue;

        const auto& vehicle_b = it_b->second;

        // Compare fields for conflicts
        if (std::abs(vehicle_a.latitude - vehicle_b.latitude) > 0.000001 ||
            std::abs(vehicle_a.longitude - vehicle_b.longitude) > 0.000001 ||
            std::abs(vehicle_a.altitude - vehicle_b.altitude) > 0.1) {
            MergeConflict conflict;
            conflict.vehicle_id = id;
            conflict.field_name = "position";
            conflict.value_a = std::to_string(vehicle_a.latitude) + "," + 
                               std::to_string(vehicle_a.longitude) + "," + 
                               std::to_string(vehicle_a.altitude);
            conflict.value_b = std::to_string(vehicle_b.latitude) + "," + 
                               std::to_string(vehicle_b.longitude) + "," + 
                               std::to_string(vehicle_b.altitude);
            conflict.timestamp_a = vehicle_a.timestamp_us;
            conflict.timestamp_b = vehicle_b.timestamp_us;
            conflict.conflict_type = SafetyStateType::FORMATION_LOST;
            conflicts.push_back(conflict);
        }

        if (std::abs(vehicle_a.battery_remaining - vehicle_b.battery_remaining) > 1.0) {
            MergeConflict conflict;
            conflict.vehicle_id = id;
            conflict.field_name = "battery";
            conflict.value_a = std::to_string(vehicle_a.battery_remaining);
            conflict.value_b = std::to_string(vehicle_b.battery_remaining);
            conflict.timestamp_a = vehicle_a.timestamp_us;
            conflict.timestamp_b = vehicle_b.timestamp_us;
            conflict.conflict_type = SafetyStateType::BATTERY_CRITICAL;
            conflicts.push_back(conflict);
        }

        if (vehicle_a.flight_mode != vehicle_b.flight_mode) {
            MergeConflict conflict;
            conflict.vehicle_id = id;
            conflict.field_name = "flight_mode";
            conflict.value_a = vehicle_a.flight_mode;
            conflict.value_b = vehicle_b.flight_mode;
            conflict.timestamp_a = vehicle_a.timestamp_us;
            conflict.timestamp_b = vehicle_b.timestamp_us;
            conflict.conflict_type = SafetyStateType::FORMATION_LOST;
            conflicts.push_back(conflict);
        }

        // Safety state conflicts
        for (const auto& safety_a : vehicle_a.safety_states) {
            auto it_b = std::find_if(vehicle_b.safety_states.begin(), vehicle_b.safety_states.end(),
                [&](const SafetyStateEntry& s) { return s.type == safety_a.type && s.source_id == safety_a.source_id; });
            
            if (it_b != vehicle_b.safety_states.end()) {
                if (safety_a.severity != it_b->severity || safety_a.acknowledged != it_b->acknowledged) {
                    MergeConflict conflict;
                    conflict.vehicle_id = id;
                    conflict.field_name = "safety_" + std::to_string(static_cast<int>(safety_a.type));
                    conflict.value_a = "sev:" + std::to_string(safety_a.severity) + " ack:" + std::to_string(safety_a.acknowledged);
                    conflict.value_b = "sev:" + std::to_string(it_b->severity) + " ack:" + std::to_string(it_b->acknowledged);
                    conflict.timestamp_a = safety_a.timestamp_us;
                    conflict.timestamp_b = it_b->timestamp_us;
                    conflict.conflict_type = safety_a.type;
                    conflicts.push_back(conflict);
                }
            } else {
                // Safety state only in one partition
                MergeConflict conflict;
                conflict.vehicle_id = id;
                conflict.field_name = "safety_" + std::to_string(static_cast<int>(safety_a.type));
                conflict.value_a = "present";
                conflict.value_b = "absent";
                conflict.timestamp_a = safety_a.timestamp_us;
                conflict.timestamp_b = 0;
                conflict.conflict_type = safety_a.type;
                conflicts.push_back(conflict);
            }
        }
    }

    return conflicts;
}

void SplitBrainMergeProtocol::resolveAllConflicts(std::vector<MergeConflict>& conflicts) {
    for (auto& conflict : conflicts) {
        resolveConflict(conflict);
    }
}

bool SplitBrainMergeProtocol::resolveConflict(MergeConflict& conflict) {
    if (conflict.resolved) return true;

    switch (conflict.resolution) {
        case MergeConflict::Resolution::LATEST_TIMESTAMP:
            conflict.resolved_value = resolveByLatestTimestamp(
                conflict.value_a, conflict.timestamp_a,
                conflict.value_b, conflict.timestamp_b);
            break;

        case MergeConflict::Resolution::HIGHEST_PRIORITY:
            // Would need priority info
            conflict.resolved_value = conflict.value_a;  // Default to A
            break;

        case MergeConflict::Resolution::MOST_RESTRICTIVE:
            conflict.resolved_value = resolveByMostRestrictive(
                conflict.value_a, conflict.value_b, conflict.conflict_type);
            break;

        case MergeConflict::Resolution::MANUAL:
            // Would require human intervention
            return false;
    }

    conflict.resolved = true;
    return true;
}

std::string SplitBrainMergeProtocol::resolveByLatestTimestamp(
    const std::string& a, uint64_t ts_a,
    const std::string& b, uint64_t ts_b) {
    return (ts_a >= ts_b) ? a : b;
}

std::string SplitBrainMergeProtocol::resolveByMostRestrictive(
    const std::string& a, const std::string& b, SafetyStateType type) {

    // For safety states, choose the more restrictive (higher severity)
    if (type == SafetyStateType::GEOFENCE || type == SafetyStateType::EMERGENCY_STOP ||
        type == SafetyStateType::BATTERY_CRITICAL || type == SafetyStateType::GPS_LOSS) {
        // Compare severity embedded in strings
        // Format: "sev:X ack:Y"
        auto extract_severity = [](const std::string& s) -> int {
            size_t pos = s.find("sev:");
            if (pos != std::string::npos) {
                pos += 4;
                size_t end = s.find(" ", pos);
                if (end != std::string::npos) {
                    return std::stoi(s.substr(pos, end - pos));
                }
            }
            return 0;
        };

        int sev_a = extract_severity(a);
        int sev_b = extract_severity(b);

        // Return more restrictive (higher severity)
        return (sev_a >= sev_b) ? a : b;
    }

    // For position/type conflicts, use latest timestamp
    return a;  // Default
}

std::string SplitBrainMergeProtocol::resolveByHighestPriority(
    const std::string& a, uint8_t priority_a,
    const std::string& b, uint8_t priority_b) {
    return (priority_a >= priority_b) ? a : b;
}

std::vector<SafetyStateEntry> SplitBrainMergeProtocol::reconcileSafetyStates(
    const std::vector<SafetyStateEntry>& states_a,
    const std::vector<SafetyStateEntry>& states_b) {

    std::map<std::pair<SafetyStateType, uint8_t>, SafetyStateEntry> merged;

    // Add all from A
    for (const auto& entry : states_a) {
        merged[{entry.type, entry.source_id}] = entry;
    }

    // Merge B entries
    for (const auto& entry : states_b) {
        auto key = std::make_pair(entry.type, entry.source_id);
        auto it = merged.find(key);
        
        if (it == merged.end()) {
            merged[key] = entry;
        } else {
            // Merge: choose more restrictive for safety
            const auto& existing = it->second;
            if (isSafetyStateMoreRestrictive(entry, existing)) {
                merged[key] = entry;
            } else if (isSafetyStateMoreRestrictive(existing, entry)) {
                // Keep existing
            } else {
                // Same restrictiveness, choose latest
                if (entry.timestamp_us > existing.timestamp_us) {
                    merged[key] = entry;
                }
            }
        }
    }

    std::vector<SafetyStateEntry> result;
    for (const auto& [_, entry] : merged) {
        result.push_back(entry);
    }

    return result;
}

bool SplitBrainMergeProtocol::isSafetyStateMoreRestrictive(
    const SafetyStateEntry& a, const SafetyStateEntry& b) const {

    // Higher severity = more restrictive
    if (a.severity != b.severity) {
        return a.severity > b.severity;
    }

    // If both acknowledged, choose the one acknowledged later (more recent)
    if (a.acknowledged && b.acknowledged) {
        return a.acknowledged_time_us > b.acknowledged_time_us;
    }

    // Unacknowledged is more restrictive (needs attention)
    if (!a.acknowledged && b.acknowledged) return true;
    if (a.acknowledged && !b.acknowledged) return false;

    // Same state, choose latest
    return a.timestamp_us > b.timestamp_us;
}

bool SplitBrainMergeProtocol::startMerge(const PartitionInfo& partition) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    if (_merge_in_progress.load()) return false;
    if (_state != PartitionState::PARTITION_DETECTED) return false;

    _state = PartitionState::MERGE_IN_PROGRESS;
    _merge_in_progress.store(true);

    _merge_thread = std::thread([this, partition]() {
        MergeResult result = performMerge(partition);
        finalizeMerge(result);
    });

    return true;
}

void SplitBrainMergeProtocol::finalizeMerge(const MergeResult& result) {
    std::lock_guard<std::mutex> lock(_mutex);

    if (!result.success) {
        _state = PartitionState::PARTITION_DETECTED;
        _merge_in_progress.store(false);
        return;
    }

    // Apply merged vehicles
    _vehicles = result.merged_vehicles;

    // Update formation if needed
    if (_config.enable_formation_recovery) {
        recoverFormation(_vehicles);
    }

    _state = PartitionState::RECOVERED;
    _merge_in_progress.store(false);
    _current_partition.reset();

    std::cout << "[MergeProtocol] Merge completed successfully: "
              << result.vehicles_merged << " vehicles, "
              << result.conflicts.size() << " conflicts resolved" << std::endl;

    // Call callback
    if (_merge_callback) {
        _merge_callback(result);
    }
}

void SplitBrainMergeProtocol::mergeVehicleStates(const VehicleStateSnapshot& a,
                                                 const VehicleStateSnapshot& b,
                                                 VehicleStateSnapshot& merged) {
    // Use latest timestamp for position
    if (a.timestamp_us >= b.timestamp_us) {
        merged.latitude = a.latitude;
        merged.longitude = a.longitude;
        merged.altitude = a.altitude;
        merged.heading = a.heading;
        merged.velocity = a.velocity;
    } else {
        merged.latitude = b.latitude;
        merged.longitude = b.longitude;
        merged.altitude = b.altitude;
        merged.heading = b.heading;
        merged.velocity = b.velocity;
    }

    // Battery: use minimum (most conservative)
    merged.battery_remaining = std::min(a.battery_remaining, b.battery_remaining);

    // Flight mode: prefer more restrictive (RTL > HOLD > AUTO > etc.)
    static const std::map<std::string, int> mode_priority = {
        {"EMERGENCY", 0}, {"RTL", 1}, {"LAND", 2}, {"HOLD", 3}, 
        {"LOITER", 4}, {"AUTO", 5}, {"GUIDED", 6}, {"MANUAL", 7}
    };
    auto it_a = mode_priority.find(a.flight_mode);
    int pri_a = (it_a != mode_priority.end()) ? it_a->second : 100;
    auto it_b = mode_priority.find(b.flight_mode);
    int pri_b = (it_b != mode_priority.end()) ? it_b->second : 100;
    merged.flight_mode = (pri_a <= pri_b) ? a.flight_mode : b.flight_mode;

    // Armed status: if either is disarmed, result is disarmed
    merged.armed = a.armed && b.armed;

    // Merge safety states
    merged.safety_states = reconcileSafetyStates(a.safety_states, b.safety_states);

    // Formation info: use latest
    if (a.timestamp_us >= b.timestamp_us) {
        merged.formation_role = a.formation_role;
        merged.formation_position = a.formation_position;
        merged.in_formation = a.in_formation;
    } else {
        merged.formation_role = b.formation_role;
        merged.formation_position = b.formation_position;
        merged.in_formation = b.in_formation;
    }

    merged.timestamp_us = std::max(a.timestamp_us, b.timestamp_us);
    merged.vehicle_id = a.vehicle_id;
}

std::optional<uint8_t> SplitBrainMergeProtocol::electFormationLeader(
    const std::map<uint8_t, VehicleStateSnapshot>& vehicles) {
    
    uint8_t best_id = 0;
    double best_score = -1.0;

    for (const auto& [id, vehicle] : vehicles) {
        if (!vehicle.is_flying || !vehicle.armed) continue;

        // Score based on battery, position (center), link quality
        double score = vehicle.battery_remaining * 0.4;
        
        // Center of swarm
        double avg_lat = 0, avg_lon = 0;
        for (const auto& [_, v] : _vehicles) {
            avg_lat += v.latitude;
            avg_lon += v.longitude;
        }
        avg_lat /= _vehicles.size();
        avg_lon /= _vehicles.size();

        double dist_to_center = std::hypot(vehicle.latitude - avg_lat, vehicle.longitude - avg_lon);
        score += (1.0 / (1.0 + dist_to_center)) * 0.3;

        // Link quality
        if (_connectivity_matrix.find(vehicle.vehicle_id) != _connectivity_matrix.end()) {
            double avg_quality = 0;
            int count = 0;
            for (const auto& [_, q] : _connectivity_matrix[vehicle.vehicle_id]) {
                avg_quality += q;
                count++;
            }
            if (count > 0) avg_quality /= count;
            score += avg_quality * 0.3;
        }

        if (score > best_score) {
            best_score = score;
            best_id = id;
        }
    }

    return best_score > 0 ? std::optional<uint8_t>(best_id) : std::nullopt;
}

bool SplitBrainMergeProtocol::recoverFormation(const std::map<uint8_t, VehicleStateSnapshot>& vehicles) {
    // Elect new leader
    auto leader_opt = electFormationLeader(vehicles);
    if (!leader_opt) return false;

    uint8_t leader_id = *leader_opt;

    // Assign formation positions
    assignFormationPositions(vehicles, leader_id);

    return true;
}

void SplitBrainMergeProtocol::assignFormationPositions(
    const std::map<uint8_t, VehicleStateSnapshot>& vehicles,
    uint8_t leader_id) {
    // Assign formation positions relative to leader
    // This would be expanded with actual formation geometry
}

void SplitBrainMergeProtocol::cancelMerge() {
    if (_merge_thread.joinable()) {
        _merge_in_progress.store(false);
        _merge_thread.join();
    }
    _state = PartitionState::PARTITION_DETECTED;
}

void SplitBrainMergeProtocol::forceMerge() {
    if (_current_partition.has_value()) {
        startMerge(_current_partition.value());
    }
}

void SplitBrainMergeProtocol::acknowledgeSafetyEvent(const SafetyStateEntry& event, uint8_t ack_by) {
    // Update safety event as acknowledged
    // Implementation would update the event in storage
}

bool SplitBrainMergeProtocol::isVehicleConnected(uint8_t vehicle_id) const {
    auto it = _connectivity_matrix.find(vehicle_id);
    if (it == _connectivity_matrix.end()) return false;

    for (const auto& [_, quality] : it->second) {
        if (quality >= _config.link_quality_threshold) return true;
    }
    return false;
}

void SplitBrainMergeProtocol::saveState() {
    // Persist state to storage
}

void SplitBrainMergeProtocol::loadState() {
    // Load state from storage
}

} // namespace swarm::companion