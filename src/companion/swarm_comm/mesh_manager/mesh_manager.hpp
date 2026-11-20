#ifndef MESH_MANAGER_HPP
#define MESH_MANAGER_HPP

#include "swarm_comm.hpp"
#include "store_forward.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>
#include <functional>

namespace swarm::comm {

// Mesh network manager with tiered failover (Wi-Fi 6E / 5G / LoRa)
class MeshManager {
public:
    struct Config {
        // Wi-Fi 6E (primary)
        bool wifi_enabled = true;
        std::string wifi_interface = "wlan0";
        std::string wifi_ssid = "swarm_mesh";
        std::string wifi_passphrase = "swarm123";
        int wifi_channel = 36;  // 6 GHz channel
        int wifi_tx_power_dbm = 20;
        
        // 5G NR (secondary)
        bool fiveg_enabled = true;
        std::string fiveg_apn = "swarm.5g";
        std::string fiveg_interface = "wwan0";
        
        // LoRa (tertiary)
        bool lora_enabled = true;
        std::string lora_device = "/dev/ttyUSB0";
        int lora_frequency_hz = 868000000;  // 868 MHz EU
        int lora_spreading_factor = 7;
        int lora_bandwidth = 125000;
        int lora_tx_power_dbm = 14;
        
        // Mesh settings
        uint16_t own_sysid = 1;
        float neighbor_timeout_s = 10.0f;
        float discovery_interval_s = 5.0f;
        float qos_update_interval_s = 1.0f;
        int max_hops = 5;
        
        // Security
        bool encryption_enabled = true;
        std::string encryption_key;  // Base64 encoded
    };
    
    struct Neighbor {
        uint16_t sysid;
        uint8_t compid;
        LinkType link_type;
        LinkQoS qos;
        std::vector<uint16_t> route;  // Multi-hop path
        uint64_t last_heard_us;
        bool is_direct;
        float trust_score;  // 0.0 - 1.0
    };
    
    struct MeshStats {
        uint64_t messages_sent = 0;
        uint64_t messages_received = 0;
        uint64_t messages_dropped = 0;
        uint64_t route_discoveries = 0;
        uint64_t route_failures = 0;
        float avg_latency_ms = 0.0f;
        size_t neighbor_count = 0;
        size_t direct_neighbors = 0;
        size_t multi_hop_neighbors = 0;
    };
    
    MeshManager(const Config& config = Config{});
    ~MeshManager();
    
    // Initialize and start mesh
    bool start();
    void stop();
    
    // Get own system ID
    uint16_t getOwnId() const { return _config.own_sysid; }
    
    // Send message to destination (handles routing)
    bool sendMessage(uint16_t dst_sysid, uint8_t dst_compid, SwarmMsgId msg_id,
                     const uint8_t* data, uint16_t len);
    
    // Broadcast message to all neighbors
    bool broadcastMessage(SwarmMsgId msg_id, const uint8_t* data, uint16_t len);
    
    // Register callback for incoming messages
    using MessageCallback = std::function<void(uint16_t src_sysid, uint8_t src_compid, 
                                               SwarmMsgId msg_id, const uint8_t* data, uint16_t len)>;
    void registerCallback(SwarmMsgId msg_id, MessageCallback cb);
    
    // Get neighbors
    const std::vector<Neighbor>& getNeighbors() const;
    std::optional<Neighbor> getNeighbor(uint16_t sysid) const;
    
    // Get mesh topology
    struct Topology {
        std::vector<Neighbor> neighbors;
        std::unordered_map<uint16_t, std::vector<uint16_t>> routes;  // dst -> path
    };
    Topology getTopology() const;
    
    // Get link QoS
    std::optional<LinkQoS> getLinkQoS(uint16_t dst_id) const;
    
    // Get mesh stats
    const MeshStats& getStats() const { return _stats; }
    
    // Get store-and-forward queue
    StoreForwardQueue& getStoreForwardQueue() { return *store_forward; }
    
    // Link quality callbacks
    using LinkQualityCallback = std::function<void(uint16_t sysid, const LinkQoS& qos)>;
    void setLinkQualityCallback(LinkQualityCallback cb) { _link_qos_cb = cb; }
    
    // Neighbor callbacks
    using NeighborCallback = std::function<void(uint16_t sysid, bool connected)>;
    void setNeighborCallback(NeighborCallback cb) { _neighbor_cb = cb; }

private:
    Config _config;
    std::atomic<bool> _running{false};
    std::thread _main_thread;
    std::thread _discovery_thread;
    std::thread _qos_thread;
    std::thread _routing_thread;
    
    std::mutex _mutex;
    std::unordered_map<uint16_t, Neighbor> _neighbors;
    std::unordered_map<SwarmMsgId, std::function<void(uint16_t, uint8_t, SwarmMsgId, const uint8_t*, uint16_t)>> _callbacks;
    
    std::unique_ptr<StoreForwardQueue> store_forward;
    
    MeshStats _stats;
    std::mutex _stats_mutex;
    
    std::function<void(uint16_t, const LinkQoS&)> _link_qos_cb;
    std::function<void(uint16_t, bool)> _neighbor_cb;
    
    // Main processing loop
    void mainLoop();
    
    // Neighbor discovery
    void discoveryLoop();
    
    // QoS monitoring
    void qosLoop();
    
    // Routing table maintenance
    void routingLoop();
    
    // Handle incoming packet
    void handlePacket(uint16_t src_sysid, uint8_t src_compid, SwarmMsgId msg_id,
                      const uint8_t* data, uint16_t len);
    
    // Send via specific link
    bool sendViaLink(const Neighbor& neighbor, SwarmMsgId msg_id, 
                     const uint8_t* data, uint16_t len);
    
    // Routing algorithms
    std::vector<uint16_t> findRoute(uint16_t dst_id);
    void updateRoutes();
    bool isRouteValid(const std::vector<uint16_t>& route) const;
    
    // Link quality estimation
    void updateLinkQoS(uint16_t neighbor_id);
    float estimateLinkQuality(uint16_t neighbor_id) const;
    
    // BATMAN-adv style routing
    void sendOriginatorMessage();
    void handleOriginatorMessage(uint16_t src_id, const uint8_t* data, uint16_t len);
    
    // Encryption/decryption
    std::vector<uint8_t> encrypt(const uint8_t* data, size_t len);
    std::vector<uint8_t> decrypt(const uint8_t* data, size_t len);
    
    // Timing
    uint64_t nowUs() const;
};

} // namespace swarm::comm

#endif // MESH_MANAGER_HPP