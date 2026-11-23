#include "leader_fail_safety.hpp"
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <cstring>
#include <algorithm>

namespace swarm::firmware {

LeaderFailSafety::LeaderFailSafety() : ModuleParams(nullptr) {
    _heartbeat_perf = perf_alloc(PC_ELAPSED, "leader_fail: heartbeat");
    _election_perf = perf_alloc(PC_ELAPSED, "leader_fail: election");
    _safety_perf = perf_alloc(PC_ELAPSED, "leader_fail: safety");
}

LeaderFailSafety::~LeaderFailSafety() {
    perf_free(_heartbeat_perf);
    perf_free(_election_perf);
    perf_free(_safety_perf);
}

int LeaderFailSafety::task_spawn(int argc, char *argv[]) {
    LeaderFailSafety *instance = new LeaderFailSafety();
    if (instance) {
        object_store(instance);
        return instance->init();
    }
    return PX4_ERROR;
}

int LeaderFailSafety::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}

int LeaderFailSafety::print_usage(const char *reason) {
    if (reason) {
        PX4_WARN("%s\n", reason);
    }
    PRINT_MODULE_USAGE_NAME("leader_fail_safety", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");
    PRINT_MODULE_USAGE_COMMAND("force_election");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

int LeaderFailSafety::init() {
    // Load configuration parameters
    loadParameters();

    // Subscribe to uORB topics
    _formation_state_sub = orb_subscribe(ORB_ID(swarm_formation_state));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _safety_sub = orb_subscribe(ORB_ID(safety));
    _swarm_election_sub = orb_subscribe(ORB_ID(swarm_election));  // Custom topic

    // Advertise uORB topics
    _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), nullptr);
    _safety_pub = orb_advertise(ORB_ID(safety), nullptr);
    _swarm_election_pub = orb_advertise(ORB_ID(swarm_election), nullptr);

    // Initialize local vehicle info
    _local_vehicle.id = _local_vehicle_id;
    _local_vehicle.election_priority = _local_priority;
    _local_vehicle.is_alive = true;
    _local_vehicle.safety_ok = true;
    _local_vehicle.last_heartbeat_us = hrt_absolute_time();

    // Add self to swarm vehicles
    _swarm_vehicles[_local_vehicle_id] = _local_vehicle;

    // Initialize performance counters
    _heartbeat_perf = perf_alloc(PC_ELAPSED, "leader_fail: heartbeat");
    _election_perf = perf_alloc(PC_ELAPSED, "leader_fail: election");
    _safety_perf = perf_alloc(PC_ELAPSED, "leader_fail: safety");

    // Subscribe to parameter updates
    param_subscribe(PARAM_UPDATE_TOPIC);

    PX4_INFO("Leader Fail Safety module initialized");
    PX4_INFO("Vehicle ID: %d, Priority: %d", _local_vehicle_id, _local_priority);

    return PX4_OK;
}

void LeaderFailSafety::loadParameters() {
    param_t param;

    // Heartbeat interval
    param = param_find("LFS_HB_INTERVAL");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _config.heartbeat_interval_ms = val;
    }

    // Election timeout
    param = param_find("LFS_ELECTION_TO");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _config.election_timeout_ms = val;
    }

    // Victory timeout
    param = param_find("LFS_VICTORY_TO");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _config.victory_timeout_ms = val;
    }

    // Heartbeat timeout
    param = param_find("LFS_HB_TO");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _config.heartbeat_timeout_ms = val;
    }

    // Max election retries
    param = param_find("LFS_MAX_RETRIES");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _config.max_election_retries = val;
    }

    // Min vehicles for election
    param = param_find("LFS_MIN_VEH");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _config.min_vehicles_for_election = val;
    }

    // Safety integration
    param = param_find("LFS_SAFETY_EN");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _config.enable_safety_integration = (val != 0);
    }

    // Local vehicle ID
    param = param_find("LFS_VEH_ID");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _local_vehicle_id = val;
    }

    // Local priority
    param = param_find("LFS_PRIORITY");
    if (param != PARAM_INVALID) {
        int32_t val;
        param_get(param, &val);
        _local_priority = val;
    }

    PX4_INFO("Config: hb_interval=%dms, election_to=%dms, victory_to=%dms, hb_to=%dms",
             _config.heartbeat_interval_ms, _config.election_timeout_ms,
             _config.victory_timeout_ms, _config.heartbeat_timeout_ms);
}

void LeaderFailSafety::run() {
    // Main loop
    while (!should_exit()) {
        uint64_t now = hrt_absolute_time();

        // Process parameter updates
        bool param_updated = false;
        orb_check(_param_update_sub, &param_updated);
        if (param_updated) {
            parameter_update_s pupdate;
            orb_copy(ORB_ID(parameter_update), _param_update_sub, &pupdate);
            updateParams();
            loadParameters();
        }

        // Process incoming messages
        bool updated = false;
        orb_check(_swarm_election_sub, &updated);
        if (updated) {
            BullyElectionMessage msg;
            orb_copy(ORB_ID(swarm_election), _swarm_election_sub, &msg);
            if (validateMessage(msg)) {
                processHeartbeat(msg);
            }
        }

        // Check formation state for leader info
        updated = false;
        orb_check(_formation_state_sub, &updated);
        if (updated) {
            swarm_formation_state_s formation;
            orb_copy(ORB_ID(swarm_formation_state), _formation_state_sub, &formation);
            _current_leader_id = formation.leader_id;
        }

        // Check safety status
        updated = false;
        orb_check(_safety_sub, &updated);
        if (updated) {
            safety_s safety;
            orb_copy(ORB_ID(safety), _safety_sub, &safety);
            // Process safety events
        }

        // Check vehicle status for local state
        updated = false;
        orb_check(_vehicle_status_sub, &updated);
        if (updated) {
            vehicle_status_s status;
            orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &status);
            _local_vehicle.is_alive = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
        }

        uint64_t now = hrt_absolute_time();

        // Send heartbeat if we're the leader or participating
        if (now - _last_heartbeat_sent_us >= _config.heartbeat_interval_ms * 1000) {
            sendHeartbeat();
            _last_heartbeat_sent_us = now;
        }

        // Check leader heartbeat timeout
        checkLeaderHeartbeat();

        // Process election state machine
        switch (_election_state) {
            case LeaderElectionState::IDLE:
                // Check if we need to start election
                if (!_is_leader && _current_leader_id != 0) {
                    if (now - _last_leader_heartbeat_us > _config.heartbeat_timeout_ms * 1000) {
                        PX4_WARN("Leader %d heartbeat timeout, starting election", _current_leader_id);
                        startElection();
                    }
                }
                break;

            case LeaderElectionState::ELECTION_IN_PROGRESS:
                // Check election timeout
                if (now - _election_start_time_us > _config.election_timeout_ms * 1000) {
                    if (isLocalVehicleHighestPriority()) {
                        declareVictory();
                    } else {
                        // Wait for victory message from higher priority vehicle
                        if (now - _election_start_time_us > _config.victory_timeout_ms * 1000) {
                            PX4_WARN("Election timeout, no victory received");
                            if (_election_retries < _config.max_election_retries) {
                                _election_retries++;
                                startElection();
                            } else {
                                PX4_ERR("Max election retries reached");
                                _election_state = LeaderElectionState::IDLE;
                            }
                        }
                    }
                }
                break;

            case LeaderElectionState::NEW_LEADER_ELECTED:
                // Wait for confirmation
                break;

            case LeaderElectionState::LEADER_CONFIRMED:
                // Normal operation as leader
                break;
        }

        // Remove stale vehicles
        removeStaleVehicles();

        // Safety monitoring
        if (_config.enable_safety_integration) {
            checkSafetyStatus();
        }

        // Update local vehicle timestamp
        _local_vehicle.last_heartbeat_us = hrt_absolute_time();
        _swarm_vehicles[_local_vehicle_id] = _local_vehicle;

        // Sleep to maintain loop rate (100 Hz)
        px4_usleep(10000);
    }
}

void LeaderFailSafety::sendHeartbeat() {
    perf_begin(_heartbeat_perf);

    BullyElectionMessage msg;
    msg.sender_id = _local_vehicle_id;
    msg.message_type = static_cast<uint8_t>(BullyElectionMessageType::HEARTBEAT);
    msg.timestamp_us = hrt_absolute_time();
    msg.priority = _local_priority;
    msg.current_leader_id = _current_leader_id;
    msg.election_sequence = _election_sequence;

    sendBullyMessage(msg);

    perf_end(_heartbeat_perf);
}

void LeaderFailSafety::processHeartbeat(const BullyElectionMessage &msg) {
    if (msg.sender_id == _local_vehicle_id) {
        return;  // Ignore own messages
    }

    // Update vehicle info
    updateVehicleState(msg.sender_id, msg);

    // Check if this is from current leader
    if (msg.sender_id == _current_leader_id) {
        _last_leader_heartbeat_us = msg.timestamp_us;
    }

    // Check if leader changed
    if (msg.current_leader_id != _current_leader_id && msg.current_leader_id != 0) {
        PX4_INFO("Leader changed from %d to %d", _current_leader_id, msg.current_leader_id);
        _current_leader_id = msg.current_leader_id;
        _is_leader = (_current_leader_id == _local_vehicle_id);
    }
}

void LeaderFailSafety::checkLeaderHeartbeat() {
    if (_current_leader_id == 0 || _current_leader_id == _local_vehicle_id) {
        return;
    }

    uint64_t now = hrt_absolute_time();
    uint64_t elapsed = now - _last_leader_heartbeat_us;

    if (elapsed > _config.heartbeat_timeout_ms * 1000) {
        PX4_WARN("Leader %d heartbeat timeout (%llu ms elapsed)",
                 _current_leader_id, elapsed / 1000);

        if (_election_state == LeaderElectionState::IDLE) {
            startElection();
        }
    }
}

void LeaderFailSafety::startElection() {
    perf_begin(_election_perf);

    if (_election_state == LeaderElectionState::ELECTION_IN_PROGRESS) {
        perf_end(_election_perf);
        return;  // Already in election
    }

    // Check minimum vehicles
    int active_count = 0;
    for (const auto &[id, veh] : _swarm_vehicles) {
        if (veh.is_alive) active_count++;
    }

    if (active_count < _config.min_vehicles_for_election) {
        PX4_WARN("Not enough vehicles for election (%d < %d)", active_count, _config.min_vehicles_for_election);
        perf_end(_election_perf);
        return;
    }

    _election_state = LeaderElectionState::ELECTION_IN_PROGRESS;
    _election_sequence++;
    _election_start_time_us = hrt_absolute_time();
    _election_retries = 0;

    PX4_INFO("Starting election (seq=%d)", _election_sequence);
    logElectionEvent("ELECTION_STARTED");

    // Send election message to all higher priority vehicles
    sendElectionMessage();

    perf_end(_election_perf);
}

void LeaderFailSafety::sendElectionMessage() {
    BullyElectionMessage msg;
    msg.sender_id = _local_vehicle_id;
    msg.message_type = static_cast<uint8_t>(BullyElectionMessageType::ELECTION);
    msg.timestamp_us = hrt_absolute_time();
    msg.priority = _local_priority;
    msg.current_leader_id = 0;  // No leader during election
    msg.election_sequence = _election_sequence;

    sendBullyMessage(msg);
    logElectionEvent("ELECTION_SENT");
}

void LeaderFailSafety::processElectionMessage(const BullyElectionMessage &msg) {
    if (msg.sender_id == _local_vehicle_id) {
        return;
    }

    // Higher priority vehicle started election
    if (msg.priority > _local_priority) {
        PX4_INFO("Higher priority vehicle %d started election, yielding", msg.sender_id);
        _election_state = LeaderElectionState::IDLE;
        _election_in_progress = false;
        return;
    }

    // Lower priority vehicle - respond with ALIVE
    if (msg.priority < _local_priority) {
        BullyElectionMessage alive_msg;
        alive_msg.sender_id = _local_vehicle_id;
        alive_msg.message_type = static_cast<uint8_t>(BullyElectionMessageType::ALIVE);
        alive_msg.timestamp_us = hrt_absolute_time();
        alive_msg.priority = _local_priority;
        alive_msg.current_leader_id = 0;
        alive_msg.election_sequence = msg.election_sequence;

        sendBullyMessage(alive_msg);
        logElectionEvent("ALIVE_SENT", msg.sender_id);
    }
}

void LeaderFailSafety::processAliveMessage(const BullyElectionMessage &msg) {
    if (msg.sender_id == _local_vehicle_id) {
        return;
    }

    // Higher priority vehicle responded alive
    if (msg.priority > _local_priority) {
        PX4_INFO("Higher priority vehicle %d is alive, yielding election", msg.sender_id);
        _election_state = LeaderElectionState::IDLE;
        _election_in_progress = false;
    }
}

void LeaderFailSafety::processVictoryMessage(const BullyElectionMessage &msg) {
    if (msg.sender_id == _local_vehicle_id) {
        return;
    }

    PX4_INFO("Victory received from vehicle %d", msg.sender_id);

    // Confirm new leader
    if (msg.priority >= _local_priority) {
        confirmNewLeader(msg.sender_id);
    }
}

void LeaderFailSafety::declareVictory() {
    perf_begin(_election_perf);

    PX4_INFO("Declaring victory, becoming new leader");
    logElectionEvent("VICTORY_DECLARED");

    BullyElectionMessage msg;
    msg.sender_id = _local_vehicle_id;
    msg.message_type = static_cast<uint8_t>(BullyElectionMessageType::VICTORY);
    msg.timestamp_us = hrt_absolute_time();
    msg.priority = _local_priority;
    msg.current_leader_id = _local_vehicle_id;
    msg.election_sequence = _election_sequence;

    sendBullyMessage(msg);

    // Confirm ourselves as leader
    confirmNewLeader(_local_vehicle_id);

    perf_end(_election_perf);
}

void LeaderFailSafety::confirmNewLeader(uint8_t new_leader_id) {
    _current_leader_id = new_leader_id;
    _is_leader = (new_leader_id == _local_vehicle_id);
    _election_state = LeaderElectionState::LEADER_CONFIRMED;
    _election_in_progress = false;
    _election_retries = 0;

    // Update formation state
    swarm_formation_state_s formation;
    formation.leader_id = new_leader_id;
    formation.timestamp_us = hrt_absolute_time();
    orb_publish(ORB_ID(swarm_formation_state), _swarm_formation_pub, &formation);

    PX4_INFO("New leader confirmed: %d", new_leader_id);
    logElectionEvent("LEADER_CONFIRMED", new_leader_id);

    // If we became leader, start sending heartbeats immediately
    if (_is_leader) {
        sendHeartbeat();
    }
}

void LeaderFailSafety::updateVehicleState(uint8_t vehicle_id, const BullyElectionMessage &msg) {
    auto &veh = _swarm_vehicles[vehicle_id];
    veh.vehicle_id = vehicle_id;
    veh.last_heartbeat_us = msg.timestamp_us;
    veh.is_alive = true;
    veh.election_priority = msg.priority;
    veh.is_leader = (msg.current_leader_id == vehicle_id);
    veh.consecutive_missed = 0;
}

void LeaderFailSafety::removeStaleVehicles() {
    uint64_t now = hrt_absolute_time();
    uint64_t stale_threshold = _config.heartbeat_timeout_ms * 3 * 1000;  // 3x heartbeat timeout

    for (auto it = _swarm_vehicles.begin(); it != _swarm_vehicles.end();) {
        if (it->first == _local_vehicle_id) {
            ++it;
            continue;
        }

        uint64_t elapsed = now - it->second.last_heartbeat_us;
        if (elapsed > stale_threshold) {
            PX4_WARN("Removing stale vehicle %d (last seen %llu ms ago)",
                     it->first, elapsed / 1000);
            it = _swarm_vehicles.erase(it);
        } else {
            ++it;
        }
    }
}

uint8_t LeaderFailSafety::getHighestPriorityVehicle() const {
    uint8_t highest_id = _local_vehicle_id;
    uint32_t highest_priority = _local_priority;

    for (const auto &[id, veh] : _swarm_vehicles) {
        if (veh.is_alive && veh.election_priority > highest_priority) {
            highest_priority = veh.election_priority;
            highest_id = id;
        }
    }

    return highest_id;
}

bool LeaderFailSafety::isLocalVehicleHighestPriority() const {
    return getHighestPriorityVehicle() == _local_vehicle_id;
}

void LeaderFailSafety::sendBullyMessage(const BullyElectionMessage &msg) {
    // Publish to uORB
    orb_publish(ORB_ID(swarm_election), _swarm_election_pub, &msg);
}

void LeaderFailSafety::publishVehicleCommand(uint16_t command, float param1, float param2, float param3) {
    vehicle_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.target_system = 0;  // Broadcast
    cmd.target_component = 0;
    cmd.source_system = _local_vehicle_id;
    cmd.source_component = 1;
    cmd.from_external = true;

    orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);
}

void LeaderFailSafety::publishSafetyEvent(uint8_t event_type, uint8_t severity, uint8_t source_id, const char *description) {
    safety_s safety{};
    safety.timestamp = hrt_absolute_time();
    safety.event_type = event_type;
    safety.severity = severity;
    safety.source_id = source_id;
    safety.description = description ? description : "";
    
    orb_publish(ORB_ID(safety), _safety_pub, &safety);
}

bool LeaderFailSafety::validateMessage(const BullyElectionMessage &msg) const {
    if (msg.sender_id == 0 || msg.sender_id > 255) return false;
    if (msg.priority == 0) return false;
    if (msg.timestamp_us == 0) return false;
    return true;
}

void LeaderFailSafety::logElectionEvent(const char *event, uint8_t vehicle_id) {
    char buf[128];
    if (vehicle_id > 0) {
        snprintf(buf, sizeof(buf), "%s (vehicle %d)", event, vehicle_id);
    } else {
        snprintf(buf, sizeof(buf), "%s", event);
    }
    PX4_INFO("%s", buf);
}

void LeaderFailSafety::triggerSafetyRTL() {
    if (_safety_triggered) return;

    PX4_WARN("Triggering RTL due to leader failure");
    publishVehicleCommand(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 0, 3);  // RTL mode
    broadcastSafetyEvent("LEADER_FAILURE", "Leader lost, triggering RTL");
    _safety_triggered = true;
}

void LeaderFailSafety::triggerEmergencyStop() {
    if (_safety_triggered) return;

    PX4_ERR("Triggering EMERGENCY STOP due to critical failure");
    publishVehicleCommand(vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION, 0, 1);
    broadcastSafetyEvent("CRITICAL_FAILURE", "Emergency stop triggered");
    _safety_triggered = true;
}

void LeaderFailSafety::broadcastSafetyEvent(const char *event_type, const char *details) {
    safety_s safety{};
    safety.timestamp = hrt_absolute_time();
    safety.event_type = 0;  // Custom event
    safety.severity = 3;    // Critical
    safety.source_id = _local_vehicle_id;
    safety.description = details ? details : event_type;

    orb_publish(ORB_ID(safety), _safety_pub, &safety);
}

void LeaderFailSafety::checkSafetyStatus() {
    perf_begin(_safety_perf);

    checkGeofenceStatus();
    checkEmergencyStopStatus();
    checkBatteryStatus();
    checkLinkQuality();

    perf_end(_safety_perf);
}

void LeaderFailSafety::checkGeofenceStatus() {
    // Check geofence status from safety topic
    // If geofence breached and we're leader, trigger RTL
    if (_is_leader) {
        // Geofence check logic
    }
}

void LeaderFailSafety::checkEmergencyStopStatus() {
    // Check if emergency stop was triggered
    if (_is_leader) {
        // Emergency stop check logic
    }
}

void LeaderFailSafety::checkBatteryStatus() {
    // Check battery levels across swarm
    // If critical battery and we're leader, trigger RTL
}

void LeaderFailSafety::checkLinkQuality() {
    // Check communication link quality
    // If link lost to multiple vehicles, consider safety action
}

} // namespace swarm::firmware