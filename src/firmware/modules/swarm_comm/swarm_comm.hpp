#ifndef SWARM_COMM_HPP
#define SWARM_COMM_HPP

#include <cstdint>
#include <vector>
#include <array>
#include <functional>
#include <optional>
#include "hal_interface.hpp"

namespace swarm::comm {

// MAVLink message IDs for swarm
enum class SwarmMsgId : uint16_t {
    SWARM_FORMATION_SET = 30000,
    SWARM_FORMATION_STATE = 30001,
    SWARM_TASK_BID = 30002,
    SWARM_TASK_AWARD = 30003,
    SWARM_AI_INFERENCE = 30004,
    SWARM_SAFETY_EVENT = 30005,
    SWARM_HUMAN_INTENT = 30006,
    SWARM_COMM_QOS = 30007,
};

// Mesh link types
enum class LinkType : uint8_t {
    WIFI_6E = 0,
    FIVE_G_NR = 1,
    LORA = 2,
    MAVLINK_SERIAL = 3,
};

// Link status
enum class LinkStatus : uint8_t {
    UP = 0,
    DEGRADED = 1,
    DOWN = 2,
    STORE_FORWARD = 3,
};

struct LinkQoS {
    uint64_t timestamp_us;
    uint16_t src_id;
    uint16_t dst_id;
    LinkType link_type;
    float latency_ms_p50;
    float latency_ms_p99;
    float packet_loss_pct;
    float bandwidth_mbps;
    float rssi_dbm;
    float snr_db;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t tx_errors;
    uint32_t rx_errors;
    std::array<uint16_t, 10> route;
    LinkStatus status;
};

struct MeshNeighbor {
    uint16_t sysid;
    LinkQoS qos;
    uint64_t last_heard_us;
    bool is_direct;
};

class MeshManager {
public:
    using MessageCallback = std::function<void(uint16_t src_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len)>;

    MeshManager(HalInterface& hal) : _hal(hal) {}
    
    bool init();
    void update();
    
    // Send message to specific drone or broadcast (dst_id=0)
    bool sendMessage(uint16_t dst_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len);
    
    // Register callback for incoming messages
    void registerCallback(SwarmMsgId msg_id, MessageCallback cb);
    
    // Get mesh topology
    const std::vector<MeshNeighbor>& getNeighbors() const { return _neighbors; }
    
    // Get link QoS for specific neighbor
    std::optional<LinkQoS> getLinkQoS(uint16_t dst_id) const;
    
    // Force mesh discovery
    void triggerDiscovery();

private:
    HalInterface& _hal;
    std::vector<MeshNeighbor> _neighbors;
    std::array<std::vector<MessageCallback>, 8> _callbacks;  // One per msg_id
    uint64_t _last_discovery_us = 0;
    
    bool sendViaMavlink(uint16_t dst_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len);
    void updateLinkQoS();
    void handleIncomingMessage(uint16_t src_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len);
};

// Consensus protocol for decentralized decision making
class ConsensusEngine {
public:
    ConsensusEngine(MeshManager& mesh) : _mesh(mesh) {}
    
    // CBBA (Consensus-Based Bundle Algorithm) for task allocation
    struct TaskBid {
        uint16_t drone_id;
        std::array<uint8_t, 16> task_id;  // UUID
        float cost;
        uint32_t sequence;
    };
    
    struct Bundle {
        std::vector<std::array<uint8_t, 16>> tasks;
        std::vector<float> costs;
    };
    
    // Run CBBA consensus round
    bool runCbbaRound(const std::vector<std::array<uint8_t, 16>>& available_tasks,
                      const std::vector<float>& drone_costs,
                      Bundle& out_bundle);
    
    // Check consensus convergence
    bool isConsensusReached() const { return _consensus_reached; }
    
    // Handle split-brain detection
    void detectSplitBrain();

private:
    MeshManager& _mesh;
    bool _consensus_reached = false;
    std::vector<TaskBid> _received_bids;
};

// Store-and-forward for degraded links
class StoreForwardQueue {
public:
    struct QueuedMessage {
        uint16_t dst_id;
        SwarmMsgId msg_id;
        std::vector<uint8_t> data;
        uint64_t enqueue_time_us;
        uint8_t priority;  // 0=highest
        uint8_t retries = 0;
    };
    
    StoreForwardQueue(size_t max_size = 1000) : _max_size(max_size) {}
    
    bool enqueue(const QueuedMessage& msg);
    std::optional<QueuedMessage> dequeue(uint16_t dst_id);
    void flush(uint16_t dst_id, std::function<bool(const QueuedMessage&)> send_fn);
    size_t size() const { return _queue.size(); }
    size_t sizeForDst(uint16_t dst_id) const;

private:
    std::vector<QueuedMessage> _queue;
    size_t _max_size;
};

} // namespace swarm::comm

#endif // SWARM_COMM_HPP