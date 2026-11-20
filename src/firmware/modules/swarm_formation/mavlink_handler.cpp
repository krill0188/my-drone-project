#include "swarm_formation.hpp"
#include "swarm_comm/mavlink_router.hpp"
#include "swarm_comm/common/crypto.hpp"
#include "mavlink.h"
#include <cstring>

namespace swarm::formation {

MavlinkFormationHandler::MavlinkFormationHandler(FormationController& controller, MeshManager& mesh)
    : _controller(controller), _mesh(mesh), _last_broadcast_us(0) {}

void MavlinkFormationHandler::handleMessage(uint16_t src_sysid, uint16_t src_compid, const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case SwarmMsgId::FORMATION_SET:
            handleFormationSet(src_sysid, msg);
            break;
        case SwarmMsgId::FORMATION_STATE:
            handleFormationState(src_sysid, msg);
            break;
        case SwarmMsgId::TASK_BID:
        case SwarmMsgId::TASK_AWARD:
        case SwarmMsgId::AI_INFERENCE:
        case SwarmMsgId::SAFETY_EVENT:
        case SwarmMsgId::HUMAN_INTENT:
        case SwarmMsgId::COMM_QOS:
            // Not handled here
            break;
    }
}

void MavlinkFormationHandler::handleFormationSet(uint16_t src_sysid, const mavlink_message_t& msg) {
    // Only process from our leader
    if (!_controller.isLeader() && src_sysid == _controller.getLeaderId()) {
        SwarmFormationSetData data;
        if (unpackFormationSet(msg, data)) {
            // Update controller with new formation set
            _controller.updateFromFormationSet(data);
        }
    }
}

void MavlinkFormationHandler::handleFormationState(uint16_t src_sysid, const mavlink_message_t& msg) {
    if (_controller.isLeader()) {
        SwarmFormationStateData data;
        if (unpackFormationState(msg, data)) {
            // Leader tracks follower states
            _controller.updateFollowerState(src_sysid, data);
        }
    }
}

bool MavlinkFormationHandler::broadcastFormationSet() {
    if (!_controller.isLeader()) return false;
    
    uint64_t now = Timestamp::nowUs();
    if (now - _last_broadcast_us < 50000) return false;  // Max 20 Hz
    
    FormationStateMsg formation_msg = _controller.getFormationStateMsg();
    
    SwarmFormationSetData data;
    data.timestamp_us = formation_msg.timestamp_us;
    data.leader_id = formation_msg.leader_id;
    data.formation_id = formation_msg.formation_id;
    data.geometry_type = formation_msg.geometry_type;
    data.spacing_m = formation_msg.spacing_m;
    data.altitude_offset_m = formation_msg.altitude_offset_m;
    data.heading_offset_deg = formation_msg.heading_offset_deg;
    data.max_lateral_error_m = formation_msg.max_lateral_error_m;
    data.max_vertical_error_m = formation_msg.max_vertical_error_m;
    data.max_velocity_error_mps = formation_msg.max_velocity_error_mps;
    data.max_yaw_error_deg = formation_msg.max_yaw_error_deg;
    data.rejoin_timeout_s = formation_msg.rejoin_timeout_s;
    data.replan_propagation_ms = formation_msg.replan_propagation_ms;
    data.num_slots = static_cast<uint8_t>(formation_msg.follower_ids.size());
    
    for (size_t i = 0; i < formation_msg.follower_ids.size() && i < 50; ++i) {
        data.follower_ids[i] = formation_msg.follower_ids[i];
        data.offsets_flu[i][0] = formation_msg.position_errors_flu[i].x;  // Actually target offsets
        data.offsets_flu[i][1] = formation_msg.position_errors_flu[i].y;
        data.offsets_flu[i][2] = formation_msg.position_errors_flu[i].z;
        data.gains_xy[i] = 1.0f;  // Would come from slot config
        data.gains_z[i] = 1.0f;
        data.gains_yaw[i] = 1.0f;
    }
    data.sequence = formation_msg.sequence;
    
    mavlink_message_t msg;
    if (packFormationSet(_mesh.getOwnId(), 1, data, msg)) {
        bool sent = _mesh.broadcastMessage(msg);
        if (sent) _last_broadcast_us = Timestamp::nowUs();
        return sent;
    }
    return false;
}

bool MavlinkFormationHandler::sendFormationState() {
    if (_controller.isLeader()) return false;
    
    FormationStateMsg formation_msg = _controller.getFormationStateMsg();
    
    SwarmFormationStateData data;
    data.timestamp_us = formation_msg.timestamp_us;
    data.follower_id = _mesh.getOwnId();
    data.formation_id = formation_msg.formation_id;
    data.position_error_flu[0] = formation_msg.position_errors_flu[0].x;  // Simplified
    data.position_error_flu[1] = formation_msg.position_errors_flu[0].y;
    data.position_error_flu[2] = formation_msg.position_errors_flu[0].z;
    data.velocity_error_flu[0] = formation_msg.velocity_errors_flu[0].x;
    data.velocity_error_flu[1] = formation_msg.velocity_errors_flu[0].y;
    data.velocity_error_flu[2] = formation_msg.velocity_errors_flu[0].z;
    data.yaw_error_deg = formation_msg.yaw_errors_deg[0];
    data.status = formation_msg.follower_statuses[0];
    data.sequence = formation_msg.sequence;
    
    mavlink_message_t msg;
    if (packFormationState(_mesh.getOwnId(), 1, data, msg)) {
        // Send to leader (unicast)
        return _mesh.sendMessage(_controller.getLeaderId(), 1, msg);
    }
    return false;
}

// Packing functions
bool MavlinkFormationHandler::packFormationSet(uint8_t sysid, uint8_t compid, 
                                                const SwarmFormationSetData& data, 
                                                mavlink_message_t& msg) {
    mavlink_swarm_formation_set_t packet;
    packet.timestamp_us = data.timestamp_us;
    packet.leader_id = data.leader_id;
    packet.formation_id = data.formation_id;
    packet.geometry_type = data.geometry_type;
    packet.spacing_m = data.spacing_m;
    packet.altitude_offset_m = data.altitude_offset_m;
    packet.heading_offset_deg = data.heading_offset_deg;
    packet.max_lateral_error_m = data.max_lateral_error_m;
    packet.max_vertical_error_m = data.max_vertical_error_m;
    packet.max_velocity_error_mps = data.max_velocity_error_mps;
    packet.max_yaw_error_deg = data.max_yaw_error_deg;
    packet.rejoin_timeout_s = data.rejoin_timeout_s;
    packet.replan_propagation_ms = data.replan_propagation_ms;
    packet.num_slots = data.num_slots;
    std::memcpy(packet.follower_ids, data.follower_ids, sizeof(data.follower_ids));
    std::memcpy(packet.offsets_flu, data.offsets_flu, sizeof(data.offsets_flu));
    std::memcpy(packet.gains_xy, data.gains_xy, sizeof(data.gains_xy));
    std::memcpy(packet.gains_z, data.gains_z, sizeof(data.gains_z));
    std::memcpy(packet.gains_yaw, data.gains_yaw, sizeof(data.gains_yaw));
    packet.sequence = data.sequence;
    
    mavlink_msg_swarm_formation_set_encode(sysid, compid, &msg, &packet);
    return true;
}

bool MavlinkFormationHandler::packFormationState(uint8_t sysid, uint8_t compid, 
                                                  const SwarmFormationStateData& data, 
                                                  mavlink_message_t& msg) {
    mavlink_swarm_formation_state_t packet;
    packet.timestamp_us = data.timestamp_us;
    packet.follower_id = data.follower_id;
    packet.formation_id = data.formation_id;
    std::memcpy(packet.position_error_flu, data.position_error_flu, sizeof(data.position_error_flu));
    std::memcpy(packet.velocity_error_flu, data.velocity_error_flu, sizeof(data.velocity_error_flu));
    packet.yaw_error_deg = data.yaw_error_deg;
    packet.status = data.status;
    packet.sequence = data.sequence;
    
    mavlink_msg_swarm_formation_state_encode(sysid, compid, &msg, &packet);
    return true;
}

bool MavlinkFormationHandler::unpackFormationSet(const mavlink_message_t& msg, SwarmFormationSetData& data) {
    mavlink_swarm_formation_set_t packet;
    mavlink_msg_swarm_formation_set_decode(&msg, &packet);
    
    data.timestamp_us = packet.timestamp_us;
    data.leader_id = packet.leader_id;
    data.formation_id = packet.formation_id;
    data.geometry_type = packet.geometry_type;
    data.spacing_m = packet.spacing_m;
    data.altitude_offset_m = packet.altitude_offset_m;
    data.heading_offset_deg = packet.heading_offset_deg;
    data.max_lateral_error_m = packet.max_lateral_error_m;
    data.max_vertical_error_m = packet.max_vertical_error_m;
    data.max_velocity_error_mps = packet.max_velocity_error_mps;
    data.max_yaw_error_deg = packet.max_yaw_error_deg;
    data.rejoin_timeout_s = packet.rejoin_timeout_s;
    data.replan_propagation_ms = packet.replan_propagation_ms;
    data.num_slots = packet.num_slots;
    std::memcpy(data.follower_ids, packet.follower_ids, sizeof(packet.follower_ids));
    std::memcpy(data.offsets_flu, packet.offsets_flu, sizeof(packet.offsets_flu));
    std::memcpy(data.gains_xy, packet.gains_xy, sizeof(packet.gains_xy));
    std::memcpy(data.gains_z, packet.gains_z, sizeof(packet.gains_z));
    std::memcpy(data.gains_yaw, packet.gains_yaw, sizeof(packet.gains_yaw));
    data.sequence = packet.sequence;
    
    return true;
}

bool MavlinkFormationHandler::unpackFormationState(const mavlink_message_t& msg, SwarmFormationStateData& data) {
    mavlink_swarm_formation_state_t packet;
    mavlink_msg_swarm_formation_state_decode(&msg, &packet);
    
    data.timestamp_us = packet.timestamp_us;
    data.follower_id = packet.follower_id;
    data.formation_id = packet.formation_id;
    std::memcpy(data.position_error_flu, packet.position_error_flu, sizeof(packet.position_error_flu));
    std::memcpy(data.velocity_error_flu, packet.velocity_error_flu, sizeof(packet.velocity_error_flu));
    data.yaw_error_deg = packet.yaw_error_deg;
    data.status = packet.status;
    data.sequence = packet.sequence;
    
    return true;
}

} // namespace swarm::formation