#include "capability_advertisement.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <chrono>
#include <random>
#include <sstream>
#include <iomanip>

namespace swarm::autonomy {

CapabilityAdvertisement::CapabilityAdvertisement(MeshManager& mesh, const Config& config)
    : _mesh(mesh), _config(config) {
}

CapabilityAdvertisement::~CapabilityAdvertisement() {
    stop();
}

void CapabilityAdvertisement::start() {
    if (_running.load()) return;
    
    _running.store(true);
    _advertisement_thread = std::thread(&CapabilityAdvertisement::advertisementLoop, this);
    _listener_thread = std::thread(&CapabilityAdvertisement::listenerLoop, this);
}

void CapabilityAdvertisement::stop() {
    if (!_running.load()) return;
    
    _running.store(false);
    
    if (_advertisement_thread.joinable()) {
        _advertisement_thread.join();
    }
    if (_listener_thread.joinable()) {
        _listener_thread.join();
    }
}

void CapabilityAdvertisement::setCapabilities(const CapabilityBitmap& caps, const std::string& platform_type) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    CapabilityBitmap old_caps = _own_capabilities;
    _own_capabilities = caps;
    _own_platform_type = platform_type;
    _firmware_version = "1.0.0";  // Would get from system
    
    if (_config.enable_on_change && checkCapabilityChanges(old_caps, caps)) {
        if (_change_cb) {
            _change_cb(_mesh.getOwnId(), old_caps, caps);
        }
    }
}

void CapabilityAdvertisement::updateCapabilities(const CapabilityBitmap& caps) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    CapabilityBitmap old_caps = _own_capabilities;
    _own_capabilities = caps;
    
    if (_config.enable_on_change && checkCapabilityChanges(old_caps, caps)) {
        // Trigger immediate re-advertisement
        if (sendAdvertisement()) {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.advertisements_sent++;
        }
        
        if (_change_cb) {
            _change_cb(_mesh.getOwnId(), old_caps, caps);
        }
    }
}

std::vector<CapabilityAdvertisement::ReceivedAdvertisement> CapabilityAdvertisement::getReceivedAdvertisements() const {
    std::lock_guard<std::mutex> lock(_received_mutex);
    std::vector<ReceivedAdvertisement> result;
    result.reserve(_received_ads.size());
    for (const auto& [id, adv] : _received_ads) {
        result.push_back(adv);
    }
    return result;
}

std::optional<CapabilityAdvertisement::ReceivedAdvertisement> CapabilityAdvertisement::getAdvertisement(uint16_t drone_id) const {
    std::lock_guard<std::mutex> lock(_received_mutex);
    auto it = _received_ads.find(drone_id);
    if (it != _received_ads.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<std::pair<uint16_t, CapabilityBitmap>> CapabilityAdvertisement::getKnownDrones() const {
    std::lock_guard<std::mutex> lock(_received_mutex);
    std::vector<std::pair<uint16_t, CapabilityBitmap>> result;
    result.reserve(_received_ads.size());
    for (const auto& [id, adv] : _received_ads) {
        result.emplace_back(id, adv.adv.capabilities);
    }
    return result;
}

const CapabilityAdvertisement::Stats& CapabilityAdvertisement::getStats() const {
    return _stats;
}

void CapabilityAdvertisement::setAdvertisementCallback(AdvertisementCallback cb) {
    _adv_cb = cb;
}

void CapabilityAdvertisement::setCapabilityChangeCallback(CapabilityChangeCallback cb) {
    _change_cb = cb;
}

void CapabilityAdvertisement::advertisementLoop() {
    // Initial delay
    std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(_config.startup_delay_s)));
    
    while (_running.load()) {
        // Send advertisement
        if (sendAdvertisement()) {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.advertisements_sent++;
        }
        
        // Wait for next interval
        std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(_config.advertisement_interval_s)));
    }
}

void CapabilityAdvertisement::listenerLoop() {
    while (_running.load()) {
        // Listen for incoming advertisements
        // Would register callback with mesh manager
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool CapabilityAdvertisement::sendAdvertisement() {
    std::lock_guard<std::mutex> lock(_mutex);
    
    Advertisement adv;
    adv.drone_id = _mesh.getOwnId();
    adv.capabilities = _own_capabilities;
    adv.platform_type = _own_platform_type;
    adv.firmware_version = _firmware_version;
    adv.timestamp_us = Timestamp::nowUs();
    adv.signature = generateSignature(adv);
    
    // Serialize and send via mesh
    // In real implementation, would serialize to bytes and send via mesh
    
    // For now, just simulate
    std::vector<uint8_t> data;
    // Would serialize adv to data
    
    return _mesh.broadcastMessage(SwarmMsgId::TASK_ALLOCATION, data.data(), data.size());
}

void CapabilityAdvertisement::listenerLoop() {
    // Register callback with mesh manager
    _mesh.registerCallback(SwarmMsgId::TASK_ALLOCATION,
        [this](uint16_t src_id, uint8_t, SwarmMsgId, const uint8_t* data, uint16_t len) {
            // Parse advertisement
            Advertisement adv;
            // Would deserialize from data
            
            // Verify and process
            if (verifyAdvertisement(adv)) {
                handleAdvertisement(adv.drone_id, adv);
            }
        });
    
    while (_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void CapabilityAdvertisement::handleAdvertisement(uint16_t src_id, const Advertisement& adv) {
    // Verify signature
    if (!verifyAdvertisement(adv)) {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.verification_failures++;
        return;
    }
    
    std::lock_guard<std::mutex> lock(_received_mutex);
    
    auto it = _received_ads.find(src_id);
    bool is_new = (it == _received_ads.end());
    
    ReceivedAdvertisement recv_adv;
    recv_adv.adv = adv;
    recv_adv.received_time_us = Timestamp::nowUs();
    recv_adv.hop_count = 1;
    
    _received_ads[src_id] = recv_adv;
    
    {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.advertisements_received++;
        if (is_new) {
            _stats.known_drones = _received_ads.size();
        } else {
            _stats.duplicates_rejected++;
        }
    }
    
    // Callback
    if (_adv_cb) {
        _adv_cb(adv, is_new);
    }
    
    // Check for capability changes
    auto old_it = _received_ads.find(src_id);
    if (old_it != _received_ads.end() && old_it != it) {
        // Compare capabilities
        if (checkCapabilityChanges(old_it->second.adv.capabilities, adv.capabilities)) {
            if (_change_cb) {
                _change_cb(src_id, old_it->second.adv.capabilities, adv.capabilities);
            }
        }
    }
}

bool CapabilityAdvertisement::verifyAdvertisement(const Advertisement& adv) const {
    if (_config.enable_on_change && adv.signature.empty()) {
        return false;
    }
    
    // Verify signature
    std::string expected = generateSignature(adv);
    return adv.signature == expected;
}

std::string CapabilityAdvertisement::generateSignature(const Advertisement& adv) const {
    // In real implementation, would use HMAC or digital signature
    std::stringstream ss;
    ss << adv.drone_id << adv.capabilities.has_rtk_gps << adv.capabilities.has_stereo_camera
       << adv.capabilities.has_lidar_2d << adv.capabilities.has_lidar_3d
       << adv.capabilities.has_uwb << adv.capabilities.has_5g_modem
       << adv.capabilities.has_ai_compute << adv.capabilities.max_flight_time_s
       << adv.capabilities.max_payload_kg << adv.capabilities.comm_tier_bitmap
       << adv.platform_type << adv.firmware_version << adv.timestamp_us;
    
    // Simple hash for demo
    std::hash<std::string> hasher;
    size_t hash = hasher(ss.str());
    
    std::stringstream ss;
    ss << std::hex << hash;
    return ss.str();
}

bool CapabilityAdvertisement::checkCapabilityChanges(const CapabilityBitmap& old_caps, const CapabilityBitmap& new_caps) {
    return old_caps.has_rtk_gps != new_caps.has_rtk_gps ||
           old_caps.has_stereo_camera != new_caps.has_stereo_camera ||
           old_caps.has_lidar_2d != new_caps.has_lidar_2d ||
           old_caps.has_lidar_3d != new_caps.has_lidar_3d ||
           old_caps.has_uwb != new_caps.has_uwb ||
           old_caps.has_5g_modem != new_caps.has_5g_modem ||
           old_caps.has_ai_compute != new_caps.has_ai_compute ||
           old_caps.max_flight_time_s != new_caps.max_flight_time_s ||
           old_caps.max_payload_kg != new_caps.max_payload_kg ||
           old_caps.comm_tier_bitmap != new_caps.comm_tier_bitmap;
}

uint64_t CapabilityAdvertisement::nowUs() const {
    return Timestamp::nowUs();
}

float CapabilityAdvertisement::elapsedS(uint64_t start_us) const {
    return (Timestamp::nowUs() - start_us) / 1e6f;
}

} // namespace swarm::autonomy