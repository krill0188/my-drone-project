#include "store_forward.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <chrono>
#include <zlib.h>

namespace swarm::comm {

StoreForwardQueue::StoreForwardQueue(MeshManager& mesh, const Config& config)
    : _mesh(mesh), _config(config) {
}

StoreForwardQueue::~StoreForwardQueue() {
    clearAll();
}

bool StoreForwardQueue::enqueue(uint16_t dst_id, SwarmMsgId msg_id, 
                                const uint8_t* data, uint16_t len,
                                uint8_t priority, bool requires_ack) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    // Check total queue limit
    if (_stats.total_messages >= _config.max_total_messages) {
        // Try to make room by dropping lowest priority messages
        if (!makeRoom()) {
            return false;
        }
    }
    
    // Check per-destination limit
    auto& queue = _queues[dst_id];
    if (queue.size() >= _config.max_queue_size) {
        // Try to drop lowest priority message
        if (!dropLowestPriority(queue)) {
            return false;
        }
    }
    
    // Compress if enabled
    std::vector<uint8_t> compressed_data;
    if (_config.enable_compression && len > 100) {
        compressed_data = compressMessage(data, len);
    } else {
        compressed_data.assign(data, data + len);
    }
    
    QueuedMessage msg;
    msg.dst_id = dst_id;
    msg.msg_id = msg_id;
    msg.data = std::move(compressed_data);
    msg.priority = priority;
    msg.enqueue_time_us = Timestamp::nowUs();
    msg.requires_ack = requires_ack;
    
    queue.push_back(std::move(msg));
    
    // Update stats
    {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.total_messages++;
        _stats.current_queue_size++;
        _stats.messages_per_destination[dst_id]++;
    }
    
    return true;
}

bool StoreForwardQueue::makeRoom() {
    // Find queue with most messages and drop lowest priority
    size_t max_size = 0;
    uint16_t target_dst = 0;
    
    for (const auto& [dst, queue] : _queues) {
        if (queue.size() > max_size) {
            max_size = queue.size();
            target_dst = dst;
        }
    }
    
    if (max_size == 0) return false;
    
    return dropLowestPriority(_queues[target_dst]);
}

bool StoreForwardQueue::dropLowestPriority(std::deque<QueuedMessage>& queue) {
    if (queue.empty()) return false;
    
    // Find lowest priority message
    auto it = std::min_element(queue.begin(), queue.end(),
        [](const QueuedMessage& a, const QueuedMessage& b) {
            if (a.priority != b.priority) return a.priority > b.priority;  // Higher number = lower priority
            return a.enqueue_time_us > b.enqueue_time_us;  // Older first
        });
    
    queue.erase(it);
    
    {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.dropped_messages++;
        if (_stats.current_queue_size > 0) _stats.current_queue_size--;
    }
    
    return true;
}

void StoreForwardQueue::processQueue() {
    std::vector<std::pair<uint16_t, std::deque<QueuedMessage>::iterator>> to_process;
    
    {
        std::lock_guard<std::mutex> lock(_mutex);
        
        // Collect messages to process
        for (auto& [dst_id, queue] : _queues) {
            auto link_it = _link_status.find(dst_id);
            bool link_up = (link_it == _link_status.end() || link_it->second.up);
            float loss = link_it != _link_status.end() ? link_it->second.packet_loss_pct : 0.0f;
            
            // Skip if link down and message requires ack
            if (!link_up) {
                // Still try store-forward if not requiring ack
            }
            
            for (auto it = queue.begin(); it != queue.end(); ) {
                const auto& msg = *it;
                
                // Check expiration
                uint64_t age = Timestamp::nowUs() - msg.enqueue_time_us;
                if (age > _config.max_message_age_us) {
                    it = queue.erase(it);
                    {
                        std::lock_guard<std::mutex> lock(_stats_mutex);
                        _stats.expired_messages++;
                        if (_stats.current_queue_size > 0) _stats.current_queue_size--;
                    }
                    continue;
                }
                
                // Check retry limit
                if (msg.retries >= msg.max_retries) {
                    it = queue.erase(it);
                    {
                        std::lock_guard<std::mutex> lock(_stats_mutex);
                        _stats.dropped_messages++;
                        if (_stats.current_queue_size > 0) _stats.current_queue_size--;
                    }
                    continue;
                }
                
                // Try to deliver
                if (tryDeliver(msg)) {
                    it = queue.erase(it);
                    {
                        std::lock_guard<std::mutex> lock(_stats_mutex);
                        _stats.delivered_messages++;
                        if (_stats.current_queue_size > 0) _stats.current_queue_size--;
                    }
                } else {
                    // Increment retry count
                    it->retries++;
                    ++it;
                }
            }
        }
    }
    
    // Cleanup expired periodically
    cleanupExpired();
}

bool StoreForwardQueue::tryDeliver(const QueuedMessage& msg) {
    // Decompress if needed
    const uint8_t* data = msg.data.data();
    uint16_t len = msg.data.size();
    
    // Try sending via mesh
    bool sent = _mesh.sendMessage(msg.dst_id, msg.msg_id, data, len);
    
    if (sent && msg.requires_ack) {
        // Would wait for ack in real implementation
    }
    
    return sent;
}

bool StoreForwardQueue::flushDestination(uint16_t dst_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _queues.find(dst_id);
    if (it == _queues.end()) return true;
    
    bool all_sent = true;
    while (!it->second.empty()) {
        if (!tryDeliver(it->second.front())) {
            all_sent = false;
            break;
        }
        it->second.pop_front();
    }
    
    if (all_sent) {
        _queues.erase(it);
    }
    
    return all_sent;
}

void StoreForwardQueue::clearDestination(uint16_t dst_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _queues.find(dst_id);
    if (it != _queues.end()) {
        size_t cleared = it->second.size();
        _queues.erase(it);
        
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.dropped_messages += cleared;
        if (_stats.current_queue_size >= cleared) {
            _stats.current_queue_size -= cleared;
        }
    }
}

void StoreForwardQueue::clearAll() {
    std::lock_guard<std::mutex> lock(_mutex);
    size_t total = 0;
    for (auto& [dst, queue] : _queues) {
        total += queue.size();
    }
    _queues.clear();
    
    std::lock_guard<std::mutex> lock(_stats_mutex);
    _stats.dropped_messages += total;
    _stats.current_queue_size = 0;
    std::fill(std::begin(_stats.messages_per_destination), 
              std::end(_stats.messages_per_destination), 0);
}

const StoreForwardQueue::QueueStats& StoreForwardQueue::getStats() const {
    return _stats;
}

size_t StoreForwardQueue::getQueueSize(uint16_t dst_id) const {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _queues.find(dst_id);
    return it != _queues.end() ? it->second.size() : 0;
}

void StoreForwardQueue::setLinkStatus(uint16_t dst_id, bool link_up, float packet_loss_pct) {
    std::lock_guard<std::mutex> lock(_mutex);
    _link_status[dst_id] = {link_up, packet_loss_pct, Timestamp::nowUs()};
}

std::vector<StoreForwardQueue::QueuedMessage> StoreForwardQueue::getPendingMessages(uint16_t dst_id) const {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _queues.find(dst_id);
    if (it == _queues.end()) return {};
    
    std::vector<QueuedMessage> result;
    result.reserve(it->second.size());
    for (const auto& msg : it->second) {
        result.push_back(msg);
    }
    return result;
}

std::vector<uint8_t> StoreForwardQueue::compressMessage(const uint8_t* data, uint16_t len) {
    // Simple zlib compression
    // In real implementation, would use zlib
    return std::vector<uint8_t>(data, data + len);
}

std::vector<uint8_t> StoreForwardQueue::decompressMessage(const std::vector<uint8_t>& data) {
    // Decompress
    return data;
}

void StoreForwardQueue::updatePriorities() {
    std::lock_guard<std::mutex> lock(_mutex);
    
    for (auto& [dst, queue] : _queues) {
        // Decay priorities over time
        for (auto& msg : queue) {
            uint64_t age = Timestamp::nowUs() - msg.enqueue_time_us;
            float age_factor = std::exp(-static_cast<float>(age) * _config.priority_decay_rate / 1e6);
            // In real implementation, would adjust priority
        }
    }
}

void StoreForwardQueue::cleanupExpired() {
    std::lock_guard<std::mutex> lock(_mutex);
    uint64_t now = Timestamp::nowUs();
    
    for (auto& [dst, queue] : _queues) {
        while (!queue.empty() && 
               now - queue.front().enqueue_time_us > _config.max_message_age_us) {
            queue.pop_front();
            {
                std::lock_guard<std::mutex> lock(_stats_mutex);
                _stats.expired_messages++;
                if (_stats.current_queue_size > 0) _stats.current_queue_size--;
            }
        }
    }
}

void StoreForwardQueue::recordDelivery(const QueuedMessage& msg, bool success) {
    // Update average delivery time
    uint64_t delivery_time = Timestamp::nowUs() - msg.enqueue_time_us;
    float delivery_ms = static_cast<float>(delivery_time) / 1000.0f;
    
    std::lock_guard<std::mutex> lock(_stats_mutex);
    _stats.avg_delivery_time_ms = 0.9f * _stats.avg_delivery_time_ms + 0.1f * delivery_ms;
}

uint64_t StoreForwardQueue::nowUs() const {
    return Timestamp::nowUs();
}

} // namespace swarm::comm