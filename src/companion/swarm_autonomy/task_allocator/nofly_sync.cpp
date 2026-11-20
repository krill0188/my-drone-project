#include "nofly_sync.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>

namespace swarm::autonomy {

NoFlyZoneSync::NoFlyZoneSync(MeshManager& mesh, const Config& config)
    : _mesh(mesh), _config(config) {
}

NoFlyZoneSync::~NoFlyZoneSync() {
    stop();
}

void NoFlyZoneSync::start() {
    if (_running.load()) return;
    
    _running.store(true);
    _sync_thread = std::thread(&NoFlyZoneSync::syncLoop, this);
    _cleanup_thread = std::thread(&NoFlyZoneSync::cleanupLoop, this);
}

void NoFlyZoneSync::stop() {
    if (!_running.load()) return;
    
    _running.store(false);
    
    if (_sync_thread.joinable()) {
        _sync_thread.join();
    }
    if (_cleanup_thread.joinable()) {
        _cleanup_thread.join();
    }
}

bool NoFlyZoneSync::addLocalZone(const NoFlyZone& zone) {
    // Verify zone
    if (_config.require_signature && zone.signature.empty()) {
        return false;
    }
    
    if (_verify_cb && !_verify_cb(zone)) {
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(_zones_mutex);
        _zones[zone.id] = zone;
    }
    
    // Broadcast to swarm
    broadcastZone(zone, ZoneUpdate::Type::ADD);
    
    {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.zones_received++;
    }
    
    if (_zone_cb) {
        _zone_cb(zone, true);
    }
    
    return true;
}

bool NoFlyZoneSync::revokeLocalZone(const std::array<uint8_t, 16>& zone_id) {
    {
        std::lock_guard<std::mutex> lock(_zones_mutex);
        auto it = _zones.find(zone_id);
        if (it == _zones.end()) return false;
        
        NoFlyZone revoked = it->second;
        revoked.valid_until_us = Timestamp::nowUs();  // Immediate revocation
        revoked.signature.clear();  // Clear signature for revocation
        
        _zones.erase(it);
    }
    
    // Broadcast revocation
    NoFlyZone revoked_zone;
    revoked_zone.id = zone_id;
    revoked_zone.valid_until_us = Timestamp::nowUs();
    broadcastZone(revoked_zone, ZoneUpdate::Type::REVOKE);
    
    {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.zones_revoked++;
    }
    
    return true;
}

std::vector<NoFlyZone> NoFlyZoneSync::getZones() const {
    std::lock_guard<std::mutex> lock(_zones_mutex);
    std::vector<NoFlyZone> zones;
    zones.reserve(_zones.size());
    for (const auto& [id, zone] : _zones) {
        zones.push_back(zone);
    }
    return zones;
}

bool NoFlyZoneSync::checkPosition(const Vector3& position) const {
    std::lock_guard<std::mutex> lock(_zones_mutex);
    
    uint64_t now = Timestamp::nowUs();
    
    for (const auto& [id, zone] : _zones) {
        if (zone.valid_until_us < Timestamp::nowUs()) continue;
        if (zone.valid_from_us > Timestamp::nowUs()) continue;
        
        // Check if position is within zone
        // Simplified: check if position is within polygon
        // Real implementation would do point-in-polygon test
        if (zone.boundary.isValid()) {
            // Point in polygon test would go here
            return true;  // Simplified
        }
    }
    return false;
}

const NoFlyZoneSync::SyncStats& NoFlyZoneSync::getStats() const {
    std::lock_guard<std::mutex> lock(_stats_mutex);
    return _stats;
}

void NoFlyZoneSync::setZoneCallback(ZoneCallback cb) {
    _zone_cb = cb;
}

void NoFlyZoneSync::setVerificationCallback(VerificationCallback cb) {
    _verify_cb = cb;
}

void NoFlyZoneSync::syncLoop() {
    using namespace std::chrono;
    
    auto period = microseconds(static_cast<long long>(_config.sync_interval_s * 1e6));
    
    while (_running.load()) {
        auto cycle_start = steady_clock::now();
        
        // Broadcast full zone list periodically
        broadcastFullZoneList();
        
        // Update stats
        {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.sync_cycles++;
        }
        
        // Maintain rate
        auto cycle_end = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(cycle_end - cycle_start);
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

void NoFlyZoneSync::cleanupLoop() {
    while (_running.load()) {
        revokeExpiredZones();
        mergeOverlappingZones();
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}

void NoFlyZoneSync::broadcastZone(const NoFlyZone& zone, ZoneUpdate::Type type) {
    if (!_mesh) return;
    
    ZoneUpdate update;
    update.type = type;
    update.zones.push_back(zone);
    update.timestamp_us = Timestamp::nowUs();
    update.source_drone = _mesh->getOwnId();  // Would get from mesh
    update.authority = zone.authority;
    
    // Serialize and send via mesh
    // In real implementation, would serialize ZoneUpdate and send via mesh
}

void NoFlyZoneSync::handleZoneUpdate(const ZoneUpdate& update) {
    for (const auto& zone : update.zones) {
        // Verify signature
        if (_config.require_signature && zone.signature.empty()) {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.zones_rejected++;
            _stats.signature_failures++;
            continue;
        }
        
        if (!verifyZone(zone)) {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.zones_rejected++;
            _stats.signature_failures++;
            continue;
        }
        
        // Verify with callback
        if (_verify_cb && !_verify_cb(zone)) {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.zones_rejected++;
            continue;
        }
        
        std::lock_guard<std::mutex> lock(_zones_mutex);
        
        if (update.type == ZoneUpdate::Type::REVOKE) {
            _zones.erase(update.zones[0].id);
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.zones_revoked++;
        } else {
            auto it = _zones.find(update.zones[0].id);
            if (it == _zones.end()) {
                _zones[update.zones[0].id] = update.zones[0];
                std::lock_guard<std::mutex> lock(_stats_mutex);
                _stats.zones_received++;
            } else {
                // Merge update
                fuseNoFlyZone(it->second, update.zones[0]);
                std::lock_guard<std::mutex> lock(_stats_mutex);
                _stats.zones_merged++;
            }
        }
        
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.zones_verified++;
    }
}

bool NoFlyZoneSync::verifyZone(const NoFlyZone& zone) const {
    if (_config.require_signature && zone.signature.empty()) {
        return false;
    }
    
    // Check authority
    if (zone.authority.empty()) {
        return false;
    }
    
    // Check validity period
    uint64_t now = Timestamp::nowUs();
    if (zone.valid_until_us < now && zone.valid_until_us > 0) {
        return false;  // Expired
    }
    
    // Check polygon validity
    if (!zone.boundary.isValid()) {
        return false;
    }
    
    // In real implementation, would verify cryptographic signature
    // using authority's public key
    
    return true;
}

void NoFlyZoneSync::mergeOverlappingZones() {
    std::lock_guard<std::mutex> lock(_zones_mutex);
    
    std::vector<std::pair<std::array<uint8_t, 16>, NoFlyZone>> zones;
    zones.reserve(_zones.size());
    for (const auto& [id, zone] : _zones) {
        zones.emplace_back(id, zone);
    }
    
    // Check pairs for overlap
    for (size_t i = 0; i < zones.size(); ++i) {
        for (size_t j = i + 1; j < zones.size(); ++j) {
            if (zonesOverlap(zones[i].second, zones[j].second)) {
                // Merge zones
                NoFlyZone merged = mergeZones(zones[i].second, zones[j].second);
                
                // Remove old zones
                _zones.erase(zones[i].first);
                _zones.erase(zones[j].first);
                
                // Add merged
                _zones[merged.id] = merged;
                
                // Update stats
                {
                    std::lock_guard<std::mutex> lock(_stats_mutex);
                    _stats.zones_merged++;
                }
                
                // Broadcast merged zone
                broadcastZone(merged, ZoneUpdate::Type::UPDATE);
            }
        }
    }
}

bool NoFlyZoneSync::zonesOverlap(const NoFlyZone& a, const NoFlyZone& b) const {
    // Simplified overlap check
    // Real implementation would check polygon intersection
    // For now, check if bounding boxes overlap
    return true;  // Simplified
}

NoFlyZone NoFlyZoneSync::mergeZones(const NoFlyZone& a, const NoFlyZone& b) {
    NoFlyZone merged = a;
    
    // Merge boundaries (union)
    // Simplified: just take union of vertices
    merged.boundary.vertices.insert(merged.boundary.vertices.end(), 
                                    b.boundary.vertices.begin(), b.boundary.vertices.end());
    
    // Take earliest start, latest end
    merged.valid_from_us = std::min(a.valid_from_us, b.valid_from_us);
    merged.valid_until_us = std::max(a.valid_until_us, b.valid_until_us);
    
    // Take more restrictive authority
    if (a.authority != b.authority) {
        merged.authority = a.authority + "," + b.authority;
    }
    
    // Generate new ID
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint8_t> dis(0, 255);
    for (auto& b : merged.id) {
        b = dis(gen);
    }
    
    return merged;
}

void NoFlyZoneSync::revokeExpiredZones() {
    uint64_t now = Timestamp::nowUs();
    std::vector<std::array<uint8_t, 16>> to_revoke;
    
    {
        std::lock_guard<std::mutex> lock(_zones_mutex);
        for (const auto& [id, zone] : _zones) {
            if (zone.valid_until_us > 0 && zone.valid_until_us < now) {
                to_revoke.push_back(id);
            }
        }
    }
    
    for (const auto& id : to_revoke) {
        revokeLocalZone(id);
    }
    
    if (!to_revoke.empty()) {
        std::lock_guard<std::mutex> lock(_stats_mutex);
        _stats.zones_revoked += to_revoke.size();
    }
}

void NoFlyZoneSync::broadcastFullZoneList() {
    std::vector<NoFlyZone> zones = getZones();
    
    if (zones.empty()) return;
    
    // Split into batches if too many
    const size_t batch_size = _config.max_zones_per_sync;
    
    for (size_t i = 0; i < zones.size(); i += batch_size) {
        size_t end = std::min(i + batch_size, zones.size());
        std::vector<NoFlyZone> batch(zones.begin() + i, zones.begin() + end);
        
        ZoneUpdate update;
        update.type = ZoneUpdate::Type::BATCH;
        update.zones = batch;
        update.timestamp_us = Timestamp::nowUs();
        update.source_drone = _mesh ? _mesh->getOwnId() : 0;
        
        // Broadcast batch
        // In real implementation, would serialize and send via mesh
    }
}

void NoFlyZoneSync::handleMavlinkMessage(uint16_t src_sysid, const mavlink_message_t& msg) {
    // Handle incoming MAVLink no-fly zone messages
    // Would decode and call handleZoneUpdate
}

uint64_t NoFlyZoneSync::nowUs() const {
    return Timestamp::nowUs();
}

} // namespace swarm::autonomy