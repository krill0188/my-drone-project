#ifndef MERGE_PROTOCOL_HPP
#define MERGE_PROTOCOL_HPP

#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <mutex>
#include <atomic>
#include <optional>
#include <functional>
#include <cstdint>
#include <algorithm>

namespace swarm::companion {

enum class PartitionState : uint8_t {
    NORMAL = 0,
    PARTITION_DETECTED = 1,
    MERGE_IN_PROGRESS = 2,
    RECONCILIATION = 3,
    RECOVERED = 3
};

enum class SafetyStateType : uint8_t {
    GEOFENCE = 0,
    EMERGENCY_STOP = 1,
    BATTERY_CRITICAL = 2,
    GPS_LOSS = 2,
    LINK_LOSS = 3,
    FORMATION_LOST = 4,
    COLLISION_WARNING = 5
};

struct SafetyStateEntry {
    SafetyStateType type;
    uint8_t source_id;
    uint64_t timestamp_us;
    uint8_t severity;      // 0=info, 1=warn, 2=error, 3=critical
    std::string description;
    std::map<std::string, std::string> metadata;
    bool acknowledged = false;
    uint64_t acknowledged_time_us = 0;
    uint8_t acknowledged_by = 0;
};

struct VehicleStateSnapshot {
    uint8_t vehicle_id;
    uint64_t timestamp_us;
    double latitude;
    double longitude;
    double altitude;
    double heading;
    double velocity;
    double battery_remaining;
    std::string flight_mode;
    bool armed;
    std::vector<SafetyStateEntry> safety_states;
    std::string formation_role;  // "leader", "follower", "standby"
    uint8_t formation_position;
    bool in_formation;
};

struct PartitionInfo {
    std::set<uint8_t> partition_a_vehicles;
    std::set<uint8_t> partition_b_vehicles;
    uint64_t detected_time_us;
    std::string detection_method;  // "heartbeat_loss", "link_quality", "explicit"
    uint32_t duration_ms = 0;
    bool resolved = false;
};

struct MergeConflict {
    uint8_t vehicle_id;
    std::string field_name;
    std::string value_a;
    std::string value_b;
    uint64_t timestamp_a;
    uint64_t timestamp_b;
    SafetyStateType conflict_type;
    enum class Resolution : uint8_t {
        LATEST_TIMESTAMP = 0,
        HIGHEST_PRIORITY = 1,
        MOST_RESTRICTIVE = 2,  // For safety: choose most restrictive
        MANUAL = 3
    } resolution = Resolution::MOST_RESTRICTIVE;
    std::string resolved_value;
    bool resolved = false;
};

struct MergeResult {
    bool success = false;
    uint64_t merge_time_us = 0;
    std::vector<MergeConflict> conflicts;
    size_t vehicles_merged = 0;
    size_t safety_states_merged = 0;
    std::string error_message;
    std::map<uint8_t, VehicleStateSnapshot> merged_vehicles;
};

struct MergeProtocolConfig {
    uint32_t partition_detection_timeout_ms = 5000;    // Time before declaring partition
    uint32_t merge_timeout_ms = 30000;                 // Max time for merge process
    uint32_t heartbeat_interval_ms = 100;              // Expected heartbeat interval
    uint32_t max_missed_heartbeats = 5;                // Missed heartbeats before partition
    bool auto_resolve_safety = true;                   // Auto-resolve safety conflicts
    bool require_manual_safety_approval = false;       // Require human for safety conflicts
    uint8_t min_vehicles_for_quorum = 2;               // Minimum for quorum
    bool enable_formation_recovery = true;             // Attempt formation recovery
    double link_quality_threshold = 0.3;               // Min link quality for connectivity
};

class SplitBrainMergeProtocol {
public:
    using SafetyCallback = std::function<void(const SafetyStateEntry&)>;
    using MergeCallback = std::function<void(const MergeResult&)>;

    SplitBrainMergeProtocol();
    ~SplitBrainMergeProtocol();

    // Initialize the merge protocol
    bool initialize(const MergeProtocolConfig& config);

    // Register callbacks
    void setSafetyCallback(SafetyCallback callback);
    void setMergeCallback(MergeCallback callback);

    // Vehicle state updates
    void updateVehicleState(const VehicleStateSnapshot& state);
    void removeVehicle(uint8_t vehicle_id);

    // Partition detection
    void checkPartitions();
    void detectPartition();
    void onPartitionDetected(const PartitionInfo& partition);

    // Merge process
    bool startMerge(const PartitionInfo& partition);
    MergeResult performMerge(const PartitionInfo& partition);
    void finalizeMerge(const MergeResult& result);

    // Conflict resolution
    std::vector<MergeConflict> detectConflicts(
        const std::map<uint8_t, VehicleStateSnapshot>& partition_a,
        const std::map<uint8_t, VehicleStateSnapshot>& partition_b
    );

    bool resolveConflict(MergeConflict& conflict);
    void resolveAllConflicts(std::vector<MergeConflict>& conflicts);

    // Safety state reconciliation
    std::vector<SafetyStateEntry> reconcileSafetyStates(
        const std::vector<SafetyStateEntry>& states_a,
        const std::vector<SafetyStateEntry>& states_b
    );

    // Formation recovery
    bool recoverFormation(const std::map<uint8_t, VehicleStateSnapshot>& vehicles);
    std::optional<uint8_t> electFormationLeader(const std::map<uint8_t, VehicleStateSnapshot>& vehicles);

    // Getters
    PartitionState getState() const { return _state; }
    const MergeProtocolConfig& getConfig() const { return _config; }
    const PartitionInfo* getCurrentPartition() const { return _current_partition ? &_current_partition : nullptr; }
    const std::map<uint8_t, VehicleStateSnapshot>& getVehicles() const { return _vehicles; }
    const std::map<uint8_t, VehicleStateSnapshot>& getLastKnownVehicles() const { return _last_known_vehicles; }

    // Manual controls
    void forceMerge();
    void cancelMerge();
    void acknowledgeSafetyEvent(const SafetyStateEntry& event, uint8_t ack_by);

private:
    // Partition detection
    void checkVehicleHeartbeats();
    void updateConnectivityMatrix();
    bool isVehicleConnected(uint8_t vehicle_id) const;

    // Merge operations
    void buildVehicleMaps(const PartitionInfo& partition,
                          std::map<uint8_t, VehicleStateSnapshot>& partition_a,
                          std::map<uint8_t, VehicleStateSnapshot>& partition_b);
    void mergeVehicleStates(const VehicleStateSnapshot& a,
                            const VehicleStateSnapshot& b,
                            VehicleStateSnapshot& merged);
    void mergeSafetyStates(const std::vector<SafetyStateEntry>& a,
                           const std::vector<SafetyStateEntry>& b,
                           std::vector<SafetyStateEntry>& merged);

    // Conflict resolution strategies
    std::string resolveByLatestTimestamp(const std::string& a, uint64_t ts_a,
                                          const std::string& b, uint64_t ts_b);
    std::string resolveByMostRestrictive(const std::string& a, const std::string& b,
                                          SafetyStateType type);
    std::string resolveByHighestPriority(const std::string& a, uint8_t priority_a,
                                          const std::string& b, uint8_t priority_b);

    // Safety state helpers
    SafetyStateEntry::Resolution getSafetyResolutionStrategy(SafetyStateType type) const;
    bool isSafetyStateMoreRestrictive(const SafetyStateEntry& a, const SafetyStateEntry& b) const;
    uint8_t getSafetySeverity(const SafetyStateEntry& entry) const;

    // Formation recovery
    void rebuildFormation(const std::map<uint8_t, VehicleStateSnapshot>& vehicles);
    void assignFormationPositions(const std::map<uint8_t, VehicleStateSnapshot>& vehicles,
                                  uint8_t leader_id);

    // State persistence
    void saveState();
    void loadState();

    // Internal state
    MergeProtocolConfig _config;
    PartitionState _state = PartitionState::NORMAL;
    std::mutex _mutex;
    std::map<uint8_t, VehicleStateSnapshot> _vehicles;
    std::map<uint8_t, VehicleStateSnapshot> _last_known_vehicles;
    std::map<uint8_t, std::map<uint8_t, double>> _connectivity_matrix;  // vehicle_id -> (target_id -> quality)
    std::map<uint8_t, uint64_t> _last_heartbeat;
    std::optional<PartitionInfo> _current_partition;
    std::vector<PartitionInfo> _partition_history;
    std::atomic<bool> _merge_in_progress{false};
    std::thread _merge_thread;
    SafetyCallback _safety_callback;
    MergeCallback _merge_callback;
    uint64_t _last_partition_check = 0;
    std::map<uint8_t, uint64_t> _missed_heartbeats;
};

} // namespace swarm::companion

#endif // MERGE_PROTOCOL_HPP