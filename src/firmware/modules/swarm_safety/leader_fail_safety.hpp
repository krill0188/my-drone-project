#ifndef LEADER_FAIL_SAFETY_HPP
#define LEADER_FAIL_SAFETY_HPP

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/swarm_formation_state.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_command.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>
#include <cstdint>
#include <array>
#include <vector>
#include <map>

namespace swarm::firmware {

enum class LeaderElectionState : uint8_t {
    IDLE = 0,
    ELECTION_IN_PROGRESS = 1,
    NEW_LEADER_ELECTED = 2,
    LEADER_CONFIRMED = 3
};

enum class BullyElectionMessageType : uint8_t {
    ELECTION = 0,
    ALIVE = 1,
    VICTORY = 2,
    HEARTBEAT = 3
};

struct LeaderElectionConfig {
    uint32_t heartbeat_interval_ms = 100;      // Leader heartbeat interval
    uint32_t election_timeout_ms = 500;        // Time to wait for higher ID responses
    uint32_t victory_timeout_ms = 1000;        // Time to wait for victory confirmation
    uint32_t heartbeat_timeout_ms = 300;       // Max missed heartbeats before election
    uint8_t max_election_retries = 3;          // Max election retry attempts
    uint8_t min_vehicles_for_election = 2;     // Minimum vehicles to trigger election
    bool enable_safety_integration = true;     // Integrate with safety monitors
};

struct SwarmVehicleInfo {
    uint8_t vehicle_id = 0;
    uint64_t last_heartbeat_us = 0;
    bool is_alive = false;
    bool is_leader = false;
    uint32_t election_priority = 0;  // Higher = more priority
    uint8_t consecutive_missed = 0;
    bool safety_ok = true;
};

struct BullyElectionMessage {
    uint8_t sender_id = 0;
    uint8_t message_type = 0;  // BullyElectionMessageType
    uint64_t timestamp_us = 0;
    uint32_t priority = 0;
    uint8_t current_leader_id = 0;
    uint8_t election_sequence = 0;
};

class LeaderFailSafety : public ModuleBase<LeaderFailSafety>, public ModuleParams {
public:
    LeaderFailSafety();
    ~LeaderFailSafety();

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    int init() override;

private:
    void run() override;

    // Heartbeat management
    void sendHeartbeat();
    void processHeartbeat(const BullyElectionMessage &msg);
    void checkLeaderHeartbeat();

    // Bully Election Algorithm
    void startElection();
    void sendElectionMessage();
    void processElectionMessage(const BullyElectionMessage &msg);
    void processAliveMessage(const BullyElectionMessage &msg);
    void processVictoryMessage(const BullyElectionMessage &msg);
    void declareVictory();
    void confirmNewLeader(uint8_t new_leader_id);

    // Safety Integration
    void triggerSafetyRTL();
    void triggerEmergencyStop();
    void broadcastSafetyEvent(const char *event_type, const char *details);
    void checkSafetyStatus();

    // State Management
    void updateVehicleState(uint8_t vehicle_id, const BullyElectionMessage &msg);
    void removeStaleVehicles();
    uint8_t getHighestPriorityVehicle() const;
    bool isLocalVehicleHighestPriority() const;

    // Configuration
    void loadParameters();

    // uORB subscriptions
    orb_subscription_t _formation_state_sub{-1};
    orb_subscription_t _vehicle_status_sub{-1};
    orb_subscription_t _safety_sub{-1};
    orb_subscription_t _swarm_election_sub{-1};  // Custom election topic

    // uORB publications
    orb_advert_t _vehicle_command_pub{nullptr};
    orb_advert_t _safety_pub{nullptr};
    orb_advert_t _swarm_election_pub{nullptr};

    // State variables
    LeaderElectionConfig _config;
    SwarmVehicleInfo _local_vehicle{};
    std::map<uint8_t, SwarmVehicleInfo> _swarm_vehicles;
    
    uint8_t _current_leader_id = 0;
    uint8_t _local_vehicle_id = 1;
    uint32_t _local_priority = 100;  // Default priority
    
    LeaderElectionState _election_state = LeaderElectionState::IDLE;
    uint8_t _election_sequence = 0;
    uint8_t _election_retries = 0;
    
    uint64_t _last_heartbeat_sent_us = 0;
    uint64_t _last_leader_heartbeat_us = 0;
    uint64_t _election_start_time_us = 0;
    
    bool _is_leader = false;
    bool _election_in_progress = false;
    bool _safety_triggered = false;

    // Performance counters
    perf_counter_t _heartbeat_perf{};
    perf_counter_t _election_perf{};
    perf_counter_t _safety_perf{};

    // Helper methods
    void sendBullyMessage(const BullyElectionMessage &msg);
    void publishVehicleCommand(uint16_t command, float param1 = 0, float param2 = 0, float param3 = 0);
    void publishSafetyEvent(uint8_t event_type, uint8_t severity, uint8_t source_id, const char *description);
    bool validateMessage(const BullyElectionMessage &msg) const;
    void logElectionEvent(const char *event, uint8_t vehicle_id = 0);

    // Safety monitor integration
    void checkGeofenceStatus();
    void checkEmergencyStopStatus();
    void checkBatteryStatus();
    void checkLinkQuality();
};

} // namespace swarm::firmware

#endif // LEADER_FAIL_SAFETY_HPP