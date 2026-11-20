#ifndef CAPABILITY_ADVERTISEMENT_HPP
#define CAPABILITY_ADVERTISEMENT_HPP

#include "capability.hpp"
#include "swarm_comm/mesh_manager.hpp"
#include <vector>
#include <mutex>
#include <chrono>

namespace swarm::autonomy {

// Heterogeneous swarm capability advertisement at startup
class CapabilityAdvertisement {
public:
    struct Config {
        float advertisement_interval_s = 30.0f;     // Periodic re-advertisement
        float startup_delay_s = 5.0f;               // Delay before first advertisement
        bool enable_periodic = true;                // Periodic re-advertisement
        bool enable_on_change = true;               // Advertise on capability change
        int max_advertisement_size = 1024;          // Max advertisement size (bytes)
    };
    
    struct Advertisement {
        uint16_t drone_id;
        CapabilityBitmap capabilities;
        std::string platform_type;      // "quad", "hex", "vtol", "fixed_wing"
        std::string firmware_version;
        uint64_t timestamp_us;
        std::string signature;          // Cryptographic signature
    };
    
    struct ReceivedAdvertisement {
        Advertisement adv;
        uint64_t received_time_us;
        int hop_count;
    };
    
    struct Stats {
        uint64_t advertisements_sent = 0;
        uint64_t advertisements_received = 0;
        uint64_t duplicates_rejected = 0;
        uint64_t verification_failures = 0;
        int known_drones = 0;
    };
    
    CapabilityAdvertisement(MeshManager& mesh, const Config& config = Config{});
    
    ~CapabilityAdvertisement();
    
    // Start/stop advertisement
    void start();
    void stop();
    
    // Set our capabilities
    void setCapabilities(const CapabilityBitmap& caps, const std::string& platform_type);
    
    // Update capabilities (triggers re-advertisement if enabled)
    void updateCapabilities(const CapabilityBitmap& caps);
    
    // Get received advertisements
    std::vector<ReceivedAdvertisement> getReceivedAdvertisements() const;
    
    // Get advertisement for specific drone
    std::optional<ReceivedAdvertisement> getAdvertisement(uint16_t drone_id) const;
    
    // Get all known drones with capabilities
    std::vector<std::pair<uint16_t, CapabilityBitmap>> getKnownDrones() const;
    
    // Get stats
    const Stats& getStats() const { return _stats; }
    
    // Callbacks
    using AdvertisementCallback = std::function<void(const Advertisement& adv, bool is_new)>;
    void setAdvertisementCallback(AdvertisementCallback cb) { _adv_cb = cb; }
    
    using CapabilityChangeCallback = std::function<void(uint16_t drone_id, const CapabilityBitmap& old_caps, const CapabilityBitmap& new_caps)>;
    void setCapabilityChangeCallback(CapabilityChangeCallback cb) { _change_cb = cb; }

private:
    MeshManager& _mesh;
    Config _config;
    
    std::atomic<bool> _running{false};
    std::thread _advertisement_thread;
    std::thread _listener_thread;
    std::mutex _mutex;
    
    CapabilityBitmap _own_capabilities;
    std::string _own_platform_type;
    std::string _firmware_version;
    
    std::unordered_map<uint16_t, ReceivedAdvertisement> _received_ads;
    std::mutex _received_mutex;
    
    Stats _stats;
    std::mutex _stats_mutex;
    
    std::function<void(const Advertisement&, bool)> _adv_cb;
    std::function<void(uint16_t, const CapabilityBitmap&, const CapabilityBitmap&)> _change_cb;
    
    // Advertisement loop
    void advertisementLoop();
    
    // Listen for incoming advertisements
    void listenerLoop();
    
    // Send advertisement
    bool sendAdvertisement();
    
    // Handle incoming advertisement
    void handleAdvertisement(uint16_t src_id, const Advertisement& adv);
    
    // Verify advertisement signature
    bool verifyAdvertisement(const Advertisement& adv) const;
    
    // Generate signature
    std::string generateSignature(const Advertisement& adv) const;
    
    // Check for capability changes
    bool checkCapabilityChanges(const CapabilityBitmap& old_caps, const CapabilityBitmap& new_caps);
    
    // Timing
    uint64_t nowUs() const;
    float elapsedS(uint64_t start_us) const;
};

} // namespace swarm::autonomy

#endif // CAPABILITY_ADVERTISEMENT_HPP