#include "mesh_manager.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <chrono>
#include <random>
#include <iostream>
#include <sstream>

namespace swarm::comm {

MeshManager::MeshManager(const Config& config) : _config(config) {
    // Initialize store-and-forward queue
    store_forward = std::make_unique<StoreForwardQueue>(*this, StoreForwardQueue::Config{});
}

MeshManager::~MeshManager() {
    stop();
}

bool MeshManager::start() {
    if (_running.load()) return true;
    
    // Initialize network interfaces
    // Would set up Wi-Fi 6E, 5G, LoRa interfaces here
    
    _running.store(true);
    
    // Start threads
    _main_thread = std::thread(&MeshManager::mainLoop, this);
    _discovery_thread = std::thread(&MeshManager::discoveryLoop, this);
    _qos_thread = std::thread(&MeshManager::qosLoop, this);
    _routing_thread = std::thread(&MeshManager::routingLoop, this);
    
    // Initialize store-and-forward
    store_forward = std::make_unique<StoreForwardQueue>(*this, StoreForwardQueue::Config{});
    
    return true;
}

void MeshManager::stop() {
    if (!_running.load()) return;
    
    _running.store(false);
    
    if (_main_thread.joinable()) _main_thread.join();
    if (_discovery_thread.joinable()) _discovery_thread.join();
    if (_qos_thread.joinable()) _qos_thread.join();
    if (_routing_thread.joinable()) _routing_thread.join();
    
    store_forward.reset();
}

bool MeshManager::sendMessage(uint16_t dst_sysid, uint8_t dst_compid, SwarmMsgId msg_id,
                              const uint8_t* data, uint16_t len) {
    // Find route
    auto route = findRoute(dst_sysid);
    if (route.empty()) {
        // Try direct if neighbor
        auto it = _neighbors.find(dst_sysid);
        if (it != _neighbors.end() && it->second.is_direct) {
            return sendViaLink(it->second, msg_id, data, len);
        }
        return false;
    }
    
    // Forward through first hop
    uint16_t next_hop = route[0];
    auto it = _neighbors.find(next_hop);
    if (it == _neighbors.end()) return false;
    
    return sendViaLink(it->second, msg_id, data, len);
}

bool MeshManager::broadcastMessage(SwarmMsgId msg_id, const uint8_t* data, uint16_t len) {
    std::lock_guard<std::mutex> lock(_mutex);
    bool any_sent = false;
    
    for (const auto& [id, neighbor] : _neighbors) {
        if (neighbor.is_direct) {
            if (sendViaLink(neighbor, msg_id, data, len)) {
                any_sent = true;
            }
        }
    }
    return any_sent;
}

void MeshManager::registerCallback(SwarmMsgId msg_id, MessageCallback cb) {
    _callbacks[msg_id] = cb;
}

const std::vector<MeshManager::Neighbor>& MeshManager::getNeighbors() const {
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<Neighbor> result;
    result.reserve(_neighbors.size());
    for (const auto& [id, neighbor] : _neighbors) {
        result.push_back(neighbor);
    }
    return result;
}

std::optional<MeshManager::Neighbor> MeshManager::getNeighbor(uint16_t sysid) const {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _neighbors.find(sysid);
    if (it != _neighbors.end()) {
        return it->second;
    }
    return std::nullopt;
}

MeshManager::Topology MeshManager::getTopology() const {
    std::lock_guard<std::mutex> lock(_mutex);
    Topology topo;
    topo.neighbors.reserve(_neighbors.size());
    for (const auto& [id, neighbor] : _neighbors) {
        topo.neighbors.push_back(neighbor);
    }
    topo.routes = _routes;  // Would need _routes member
    return topo;
}

std::optional<LinkQoS> MeshManager::getLinkQoS(uint16_t dst_id) const {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _neighbors.find(dst_id);
    if (it != _neighbors.end()) {
        return it->second.qos;
    }
    return std::nullopt;
}

const MeshManager::MeshStats& MeshManager::getStats() const {
    return _stats;
}

void MeshManager::setLinkQualityCallback(LinkQualityCallback cb) {
    _link_qos_cb = cb;
}

void MeshManager::setNeighborCallback(NeighborCallback cb) {
    _neighbor_cb = cb;
}

void MeshManager::mainLoop() {
    while (_running.load()) {
        // Process incoming packets
        // In real implementation, would read from network interfaces
        
        // Update stats
        {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.neighbor_count = _neighbors.size();
            _stats.direct_neighbors = 0;
            _stats.multi_hop_neighbors = 0;
            for (const auto& [id, n] : _neighbors) {
                if (n.is_direct) _stats.direct_neighbors++;
                else _stats.multi_hop_neighbors++;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MeshManager::discoveryLoop() {
    auto period = std::chrono::duration<float>(_config.discovery_interval_s);
    
    while (_running.load()) {
        auto start = std::chrono::steady_clock::now();
        
        // Send originator message (BATMAN-adv style)
        sendOriginatorMessage();
        
        // Discover neighbors on each interface
        // Would scan Wi-Fi, 5G, LoRa
        
        // Check neighbor timeouts
        auto now = Timestamp::nowUs();
        float timeout_us = _config.neighbor_timeout_s * 1e6;
        
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (auto it = _neighbors.begin(); it != _neighbors.end(); ) {
                uint64_t age = now - it->second.last_heard_us;
                if (age > static_cast<uint64_t>(timeout_us)) {
                    // Neighbor timed out
                    if (_neighbor_cb) {
                        _neighbor_cb(it->first, false);
                    }
                    it = _neighbors.erase(it);
                } else {
                    ++it;
                }
            }
        }
        
        // Maintain discovery interval
        auto end = std::chrono::steady_clock::now();
        auto elapsed = end - std::chrono::steady_clock::now();
        if (elapsed < std::chrono::microseconds(static_cast<long long>(_config.discovery_interval_s * 1e6))) {
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<long long>(_config.discovery_interval_s * 1e6)) - elapsed);
        }
    }
}

void MeshManager::qosLoop() {
    auto period = std::chrono::duration<float>(_config.qos_update_interval_s);
    
    while (_running.load()) {
        auto start = std::chrono::steady_clock::now();
        
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (auto& [id, neighbor] : _neighbors) {
                updateLinkQoS(neighbor.sysid);
                
                if (_link_qos_cb) {
                    _link_qos_cb(neighbor.sysid, neighbor.qos);
                }
            }
        }
        
        auto end = std::chrono::steady_clock::now();
        auto elapsed = end - std::chrono::steady_clock::now();
        if (elapsed < std::chrono::microseconds(static_cast<long long>(_config.qos_update_interval_s * 1e6))) {
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<long long>(_config.qos_update_interval_s * 1e6)) - elapsed);
        }
    }
}

void MeshManager::routingLoop() {
    while (_running.load()) {
        updateRoutes();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void MeshManager::handlePacket(uint16_t src_sysid, uint8_t src_compid, SwarmMsgId msg_id,
                               const uint8_t* data, uint16_t len) {
    // Update neighbor last heard
    {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _neighbors.find(src_sysid);
        if (it != _neighbors.end()) {
            it->second.last_heard_us = Timestamp::nowUs();
        } else {
            // New neighbor
            Neighbor n;
            n.sysid = src_sysid;
            n.compid = src_compid;
            n.last_heard_us = Timestamp::nowUs();
            n.is_direct = true;  // Assume direct if we receive packet
            n.trust_score = 0.5f;
            _neighbors[src_sysid] = n;
            
            if (_neighbor_cb) {
                _neighbor_cb(src_sysid, true);
            }
        }
    }
    
    // Decrypt if needed
    std::vector<uint8_t> decrypted;
    if (_config.encryption_enabled) {
        decrypted = decrypt(data, len);
        data = decrypted.data();
        len = decrypted.size();
    }
    
    // Handle message
    auto it = _callbacks.find(msg_id);
    if (it != _callbacks.end()) {
        it->second(src_sysid, 1, msg_id, data, len);  // compid = 1 for simplicity
    }
    
    // Handle special messages
    switch (msg_id) {
        case SwarmMsgId::TASK_BID:
        case SwarmMsgId::TASK_AWARD:
        case SwarmMsgId::TASK_ALLOCATION:  // Bundle
        case SwarmMsgId::AI_INFERENCE:
        case SwarmMsgId::SAFETY_EVENT:
        case SwarmMsgId::HUMAN_INTENT:
        case SwarmMsgId::COMM_QOS:
        case SwarmMsgId::ORIGINATOR:
        case SwarmMsgId::ROUTE_REQUEST:
        case SwarmMsgId::ROUTE_REPLY:
            // Would handle specific messages
            break;
    }
    
    // Update stats
    {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.messages_received++;
    }
}

bool MeshManager::sendViaLink(const Neighbor& neighbor, SwarmMsgId msg_id, 
                              const uint8_t* data, uint16_t len) {
    // Encrypt if needed
    std::vector<uint8_t> encrypted;
    const uint8_t* send_data = data;
    uint16_t send_len = len;
    
    if (_config.encryption_enabled) {
        encrypted = encrypt(data, len);
        send_data = encrypted.data();
        send_len = encrypted.size();
    }
    
    // In real implementation, would send via appropriate interface
    // (Wi-Fi, 5G, LoRa) based on neighbor.link_type
    
    // For now, just update stats
    {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.messages_sent++;
    }
    
    return true;  // Simplified
}

std::vector<uint16_t> MeshManager::findRoute(uint16_t dst_id) {
    // Simple BFS routing
    std::unordered_map<uint16_t, uint16_t> parent;
    std::queue<uint16_t> q;
    
    q.push(_config.own_sysid);
    parent[_config.own_sysid] = 0;
    
    while (!q.empty()) {
        uint16_t current = q.front(); q.pop();
        
        if (current == dst_id) {
            // Reconstruct path
            std::vector<uint16_t> path;
            uint16_t node = dst_id;
            while (node != 0 && parent.count(node)) {
                path.push_back(node);
                node = parent[node];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // Get neighbors of current
        auto it = _neighbors.find(current);
        if (it != _neighbors.end()) {
            // In real implementation, would use neighbor's neighbor table
            // For now, assume all neighbors are directly connected
            for (const auto& [id, n] : _neighbors) {
                if (!parent.count(n.sysid)) {
                    parent[n.sysid] = current;
                    q.push(n.sysid);
                }
            }
        }
    }
    
    return {};  // No route found
}

void MeshManager::updateRoutes() {
    // In full implementation, would run BATMAN-adv or similar
    // For now, simple direct routing
}

bool MeshManager::isRouteValid(const std::vector<uint16_t>& route) const {
    if (route.empty()) return false;
    
    for (uint16_t id : route) {
        auto it = _neighbors.find(id);
        if (it == _neighbors.end() || it->second.qos.packet_loss_pct > 0.5f) {
            return false;
        }
    }
    return true;
}

void MeshManager::updateLinkQoS(uint16_t neighbor_id) {
    auto it = _neighbors.find(neighbor_id);
    if (it == _neighbors.end()) return;
    
    Neighbor& n = it->second;
    
    // In real implementation, would measure:
    // - Packet loss rate
    // - Latency
    // - RSSI/SNR
    // - Bandwidth
    
    // Simulated QoS update
    n.qos.latency_ms_p50 = 5.0f + (rand() % 10) * 0.5f;
    n.qos.latency_ms_p99 = n.qos.latency_ms_p50 * 2.0f;
    n.qos.packet_loss_pct = (rand() % 100) / 1000.0f;  // 0-10%
    n.qos.bandwidth_mbps = 50.0f + (rand() % 50) * 1.0f;
    n.qos.rssi_dbm = -50.0f - (rand() % 30);
    n.qos.snr_db = 20.0f + (rand() % 10);
    
    // Update status
    if (n.qos.packet_loss_pct > 0.3f) {
        n.qos.status = LinkStatus::DEGRADED;
    } else if (n.qos.packet_loss_pct > 0.5f) {
        n.qos.status = LinkStatus::DOWN;
    } else {
        n.qos.status = LinkStatus::UP;
    }
    
    n.qos.last_update_us = Timestamp::nowUs();
}

float MeshManager::estimateLinkQuality(uint16_t neighbor_id) const {
    auto it = _neighbors.find(neighbor_id);
    if (it == _neighbors.end()) return 0.0f;
    
    const auto& n = it->second;
    
    // Combine metrics into quality score
    float quality = 1.0f;
    quality *= (1.0f - n.qos.packet_loss_pct);
    quality *= std::min(1.0f, 100.0f / (n.qos.latency_ms_p50 + 1.0f));
    quality *= std::min(1.0f, (n.qos.rssi_dbm + 100.0f) / 50.0f);
    
    return std::clamp(quality, 0.0f, 1.0f);
}

void MeshManager::sendOriginatorMessage() {
    // BATMAN-adv style originator message
    // Broadcast our presence and sequence number
}

void MeshManager::handleOriginatorMessage(uint16_t src_id, const uint8_t* data, uint16_t len) {
    // Process BATMAN-adv originator message
    // Update routing tables
}

std::vector<uint8_t> MeshManager::encrypt(const uint8_t* data, size_t len) {
    // In real implementation, would use AES-GCM or ChaCha20-Poly1305
    // For now, just return copy
    return std::vector<uint8_t>(data, data + len);
}

std::vector<uint8_t> MeshManager::decrypt(const uint8_t* data, size_t len) {
    // Decrypt
    return std::vector<uint8_t>(data, data + len);
}

uint64_t MeshManager::nowUs() const {
    return Timestamp::nowUs();
}

} // namespace swarm::comm