#ifndef MAVLINK_ROUTER_HPP
#define MAVLINK_ROUTER_HPP

#include <cstdint>
#include <vector>
#include <array>
#include <functional>
#include <unordered_map>
#include "mavlink.h"

namespace swarm::comm {

// MAVLink message routing for mesh networks
class MavlinkRouter {
public:
    using MessageCallback = std::function<void(uint8_t sysid, uint8_t compid, const mavlink_message_t&)>;
    
    struct RouteEntry {
        uint8_t next_hop_sysid;
        uint8_t next_hop_compid;
        uint8_t hops;
        uint64_t last_update_us;
        bool valid;
    };
    
    MavlinkRouter(uint8_t own_sysid, uint8_t own_compid = 1)
        : _own_sysid(own_sysid), _own_compid(own_compid) {}
    
    // Initialize router
    bool init();
    
    // Send message to destination (handles routing)
    bool sendMessage(uint8_t dst_sysid, uint8_t dst_compid, 
                     const mavlink_message_t& msg);
    
    // Broadcast message to all neighbors
    bool broadcastMessage(const mavlink_message_t& msg);
    
    // Handle incoming message (from serial/UDP)
    void handleIncoming(uint8_t src_sysid, uint8_t src_compid,
                        const mavlink_message_t& msg);
    
    // Register callback for specific message ID
    void registerCallback(uint16_t msg_id, MessageCallback cb);
    
    // Get route to destination
    std::optional<RouteEntry> getRoute(uint8_t dst_sysid) const;
    
    // Update link quality (for routing decisions)
    void updateLinkQuality(uint8_t neighbor_sysid, float quality);
    
    // Periodic update (call at 10 Hz)
    void update();
    
    // Get all known routes
    const std::unordered_map<uint8_t, RouteEntry>& getRoutes() const {
        return _routes;
    }
    
    // Get neighbor list
    std::vector<uint8_t> getNeighbors() const;

private:
    uint8_t _own_sysid;
    uint8_t _own_compid;
    
    // Routing table: dst_sysid -> route entry
    std::unordered_map<uint8_t, RouteEntry> _routes;
    
    // Message callbacks
    std::unordered_map<uint16_t, std::vector<MessageCallback>> _callbacks;
    
    // Neighbor link quality
    std::unordered_map<uint8_t, float> _link_quality;
    
    // Sequence numbers for deduplication
    std::unordered_map<uint16_t, uint16_t> _seen_sequences;
    
    // BATMAN-adv style originator messages
    void sendOriginatorMessage();
    void handleOriginatorMessage(uint8_t src_sysid, const mavlink_message_t& msg);
    
    // Route discovery
    void discoverRoute(uint8_t dst_sysid);
    
    // Forward message to next hop
    bool forwardMessage(uint8_t dst_sysid, const mavlink_message_t& msg);
    
    // Check if we should forward
    bool shouldForward(uint8_t src_sysid, uint16_t msg_id, uint16_t seq);
    
    // Update route from received message
    void updateRoute(uint8_t src_sysid, uint8_t prev_hop_sysid, uint8_t hops);
};

// Swarm-specific MAVLink message IDs
namespace SwarmMsgId {
    constexpr uint16_t FORMATION_SET = 30000;
    constexpr uint16_t FORMATION_STATE = 30001;
    constexpr uint16_t TASK_BID = 30002;
    constexpr uint16_t TASK_AWARD = 30003;
    constexpr uint16_t AI_INFERENCE = 30004;
    constexpr uint16_t SAFETY_EVENT = 30005;
    constexpr uint16_t HUMAN_INTENT = 30006;
    constexpr uint16_t COMM_QOS = 30007;
    constexpr uint16_t ORIGINATOR = 30008;  // BATMAN-adv style
    constexpr uint16_t ROUTE_REQUEST = 30009;
    constexpr uint16_t ROUTE_REPLY = 30010;
}

// Helper to pack swarm messages into MAVLink
class SwarmMessagePacker {
public:
    // Pack formation set message
    static bool packFormationSet(uint8_t sysid, uint8_t compid,
                                 const FormationSetData& data,
                                 mavlink_message_t& msg);
    
    // Pack formation state message
    static bool packFormationState(uint8_t sysid, uint8_t compid,
                                   const FormationStateData& data,
                                   mavlink_message_t& msg);
    
    // Pack task bid message
    static bool packTaskBid(uint8_t sysid, uint8_t compid,
                            const TaskBidData& data,
                            mavlink_message_t& msg);
    
    // Pack AI inference message
    static bool packAiInference(uint8_t sysid, uint8_t compid,
                                const AiInferenceData& data,
                                mavlink_message_t& msg);
    
    // Pack safety event message
    static bool packSafetyEvent(uint8_t sysid, uint8_t compid,
                                const SafetyEventData& data,
                                mavlink_message_t& msg);
};

// Data structures matching contracts/mavlink/swarm_messages.proto

struct FormationSetData {
    uint32_t timestamp_us;
    uint16_t leader_id;
    uint32_t formation_id;
    uint8_t geometry_type;
    float spacing_m;
    float altitude_offset_m;
    float heading_offset_deg;
    float max_lateral_error_m;
    float max_vertical_error_m;
    float max_velocity_error_mps;
    float max_yaw_error_deg;
    float rejoin_timeout_s;
    float replan_propagation_ms;
    uint8_t num_slots;
    uint16_t follower_ids[50];
    float offsets_flu[50][3];
    float gains_xy[50];
    float gains_z[50];
    float gains_yaw[50];
    uint32_t sequence;
};

struct FormationStateData {
    uint32_t timestamp_us;
    uint16_t follower_id;
    uint32_t formation_id;
    float position_error_flu[3];
    float velocity_error_flu[3];
    float yaw_error_deg;
    uint8_t status;
    uint32_t sequence;
};

struct TaskBidData {
    uint32_t timestamp_us;
    uint16_t drone_id;
    uint8_t task_id[16];
    float cost;
    bool has_rtk_gps;
    bool has_stereo_camera;
    bool has_lidar_2d;
    bool has_lidar_3d;
    bool has_uwb;
    bool has_5g_modem;
    bool has_ai_compute;
    uint16_t max_flight_time_s;
    float max_payload_kg;
    uint8_t comm_tier_bitmap;
    uint32_t sequence;
};

struct AiInferenceData {
    uint32_t timestamp_us;
    uint16_t drone_id;
    uint8_t type;
    uint16_t class_id;
    char class_name[32];
    float confidence;
    float position_enu[3];
    float bbox_size[3];
    float bbox_orientation[4];
    uint32_t latency_ms;
    char model_name[32];
    char model_version[16];
    char runtime[16];
    uint8_t threat_level;
    float relative_velocity[3];
    float time_to_collision_s;
    bool requires_immediate_avoidance;
    float slope_deg;
    float clearance_radius_m;
    uint8_t surface_type;
    float firmness_score;
    uint8_t fault_type;
    uint8_t severity;
    char affected_component[32];
    float degradation_pct;
    bool requires_replan;
    uint8_t recommended_action;
};

struct SafetyEventData {
    uint32_t timestamp_us;
    uint16_t source_id;
    uint8_t type;
    uint8_t severity;
    float position_enu[3];
    char details[128];
    uint32_t sequence;
};

} // namespace swarm::comm

#endif // MAVLINK_ROUTER_HPP