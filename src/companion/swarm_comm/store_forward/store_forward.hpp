#ifndef STORE_FORWARD_HPP
#define STORE_FORWARD_HPP

#include "swarm_comm.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <mutex>
#include <chrono>

namespace swarm::comm {

// Store-and-forward message queue for degraded mesh links
class StoreForwardQueue {
public:
    struct Config {
        size_t max_queue_size = 1000;           // Max messages per destination
        uint64_t max_message_age_us = 30000000; // 30 seconds max age
        size_t max_total_messages = 10000;      // Total queue limit
        float priority_decay_rate = 0.1f;       // Priority decay per second
        bool enable_compression = true;         // Compress messages
    };
    
    struct QueuedMessage {
        uint16_t dst_id;
        SwarmMsgId msg_id;
        std::vector<uint8_t> data;
        uint8_t priority;          // 0 = highest
        uint64_t enqueue_time_us;
        uint8_t retries = 0;
        uint8_t max_retries = 3;
        bool requires_ack = false;
        std::string metadata;
    };
    
    struct QueueStats {
        size_t total_messages = 0;
        size_t delivered_messages = 0;
        size_t expired_messages = 0;
        size_t dropped_messages = 0;
        size_t current_queue_size = 0;
        float avg_delivery_time_ms = 0.0f;
        size_t messages_per_destination[65536] = {0};
    };
    
    StoreForwardQueue(MeshManager& mesh, const Config& config = Config{});
    
    ~StoreForwardQueue();
    
    // Enqueue message for delivery
    bool enqueue(uint16_t dst_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len,
                 uint8_t priority = 0, bool requires_ack = false);
    
    // Try to deliver queued messages
    void processQueue();
    
    // Force deliver to specific destination
    bool flushDestination(uint16_t dst_id);
    
    // Clear queue for destination
    void clearDestination(uint16_t dst_id);
    
    // Clear all queues
    void clearAll();
    
    // Get queue statistics
    const QueueStats& getStats() const { return _stats; }
    
    // Get queue size for destination
    size_t getQueueSize(uint16_t dst_id) const;
    
    // Set link status (called by mesh manager)
    void setLinkStatus(uint16_t dst_id, bool link_up, float packet_loss_pct);
    
    // Get pending messages for destination
    std::vector<QueuedMessage> getPendingMessages(uint16_t dst_id) const;

private:
    MeshManager& _mesh;
    Config _config;
    
    std::mutex _mutex;
    std::unordered_map<uint16_t, std::deque<QueuedMessage>> _queues;
    
    struct LinkStatus {
        bool up = true;
        float packet_loss_pct = 0.0f;
        uint64_t last_update_us = 0;
    };
    std::unordered_map<uint16_t, LinkStatus> _link_status;
    
    struct Stats _stats;
    std::mutex _stats_mutex;
    
    // Message delivery
    bool tryDeliver(const QueuedMessage& msg);
    
    // Compression
    std::vector<uint8_t> compressMessage(const uint8_t* data, uint16_t len);
    std::vector<uint8_t> decompressMessage(const std::vector<uint8_t>& data);
    
    // Priority management
    void updatePriorities();
    
    // Cleanup expired messages
    void cleanupExpired();
    
    // Statistics
    void recordDelivery(const QueuedMessage& msg, bool success);
    
    // Timing
    uint64_t nowUs() const;
};

} // namespace swarm::comm

#endif // STORE_FORWARD_HPP