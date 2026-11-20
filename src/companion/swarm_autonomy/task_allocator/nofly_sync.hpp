#ifndef NOFLY_SYNC_HPP
#define NOFLY_SYNC_HPP

#include "swarm_map.hpp"
#include "swarm_comm/mesh_manager.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>
#include <functional>

namespace swarm::autonomy {

// No-fly zone synchronization with cryptographic verification
class NoFlyZoneSync {
public:
    struct Config {
        float sync_interval_s = 60.0f;           // Sync interval
        float max_zone_age_s = 3600.0f;          // Max zone age (1 hour)
        bool require_signature = true;           // Require cryptographic signature
        float zone_merge_threshold = 0.1f;       // Merge zones if overlap > threshold
        int max_zones_per_sync = 100;            // Max zones per sync message
        bool enable_auto_revocation = true;      // Auto-revoke expired zones
    };
    
    struct SyncStats {
        uint64_t zones_received = 0;
        uint64_t zones_verified = 0;
        uint64_t zones_rejected = 0;
        uint64_t zones_merged = 0;
        uint64_t zones_revoked = 0;
        uint64_t sync_cycles = 0;
        uint64_t signature_failures = 0;
    };
    
    struct ZoneUpdate {
        enum class Type {
            ADD,
            UPDATE,
            REVOKE,
            BATCH
        };
        Type type;
        std::vector<NoFlyZone> zones;
        uint64_t timestamp_us;
        uint16_t source_drone;
        std::string authority;
    };
    
    NoFlyZoneSync(MeshManager& mesh, const Config& config = Config{});
    
    ~NoFlyZoneSync();
    
    // Start/stop sync
    void start();
    void stop();
    
    // Add local no-fly zone
    bool addLocalZone(const NoFlyZone& zone);
    
    // Revoke local zone
    bool revokeLocalZone(const std::array<uint8_t, 16>& zone_id);
    
    // Get current zones
    std::vector<NoFlyZone> getZones() const;
    
    // Check if position is in no-fly zone
    bool checkPosition(const Vector3& position) const;
    
    // Get sync stats
    const SyncStats& getStats() const { return _stats; }
    
    // Callbacks
    using ZoneCallback = std::function<void(const NoFlyZone& zone, bool added)>;
    void setZoneCallback(ZoneCallback cb) { _zone_cb = cb; }
    
    using VerificationCallback = std::function<bool(const NoFlyZone& zone)>;
    void setVerificationCallback(VerificationCallback cb) { _verify_cb = cb; }

private:
    MeshManager& _mesh;
    Config _config;
    
    std::atomic<bool> _running{false};
    std::thread _sync_thread;
    std::thread _cleanup_thread;
    
    std::mutex _zones_mutex;
    std::unordered_map<std::array<uint8_t, 16>, NoFlyZone, ArrayHash> _zones;
    
    SyncStats _stats;
    std::mutex _stats_mutex;
    
    ZoneCallback _zone_cb;
    VerificationCallback _verify_cb;
    
    // Sync loop
    void syncLoop();
    
    // Cleanup expired zones
    void cleanupLoop();
    
    // Broadcast zone update
    void broadcastZone(const NoFlyZone& zone, ZoneUpdate::Type type);
    
    // Handle incoming zone update
    void handleZoneUpdate(const ZoneUpdate& update);
    
    // Verify zone signature
    bool verifyZone(const NoFlyZone& zone) const;
    
    // Merge overlapping zones
    void mergeOverlappingZones();
    
    // Revoke expired zones
    void revokeExpiredZones();
    
    // Broadcast full zone list
    void broadcastFullZoneList();
    
    // Handle incoming MAVLink message
    void handleMavlinkMessage(uint16_t src_sysid, const mavlink_message_t& msg);
    
    // Hash for UUID array
    struct ArrayHash {
        size_t operator()(const std::array<uint8_t, 16>& arr) const noexcept {
            size_t hash = 0;
            for (uint8_t b : arr) {
                hash = hash * 31 + b;
            }
            return hash;
        }
    };
    
    // Timing
    uint64_t nowUs() const;
};

} // namespace swarm::autonomy

#endif // NOFLY_SYNC_HPP