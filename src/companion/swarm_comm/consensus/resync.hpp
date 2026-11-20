#ifndef RESYNC_HPP
#define RESYNC_HPP

#include "consensus.hpp"
#include "swarm_comm/mesh_manager.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>
#include <chrono>

namespace swarm::comm {

// Consensus re-synchronization after link restoration
class ConsensusResync {
public:
    struct Config {
        float max_resync_time_s = 10.0f;          // Max time to complete resync
        float state_request_timeout_s = 2.0f;     // Timeout for state requests
        int max_state_request_retries = 3;
        bool enable_incremental_sync = true;      // Sync only changes
        float state_diff_threshold = 0.01f;       // Min difference to sync
    };
    
    struct ResyncState {
        enum class Phase {
            IDLE,
            REQUESTING_STATE,
            RECEIVING_STATE,
            MERGING,
            VERIFYING,
            COMPLETE,
            FAILED
        };
        
        Phase phase = Phase::IDLE;
        uint32_t resync_id = 0;
        uint16_t peer_id = 0;
        uint64_t start_time_us = 0;
        uint64_t deadline_us = 0;
        std::vector<uint8_t> expected_state_hash;
        std::vector<uint8_t> received_state_hash;
        int retries = 0;
        std::string error_message;
    };
    
    struct ResyncStats {
        uint64_t total_resyncs = 0;
        uint64_t successful_resyncs = 0;
        uint64_t failed_resyncs = 0;
        float avg_resync_time_s = 0.0f;
        uint64_t bytes_synced = 0;
    };
    
    ConsensusResync(ConsensusEngine& consensus, MeshManager& mesh, 
                    const Config& config = Config{});
    
    ~ConsensusResync();
    
    // Trigger resync with peer
    bool triggerResync(uint16_t peer_id);
    
    // Trigger full resync with all peers
    void triggerFullResync();
    
    // Handle incoming state data
    void handleStateData(uint16_t src_id, const std::vector<uint8_t>& state_data);
    
    // Handle state request
    void handleStateRequest(uint16_t requester_id, uint32_t resync_id);
    
    // Check if resync in progress
    bool isResyncing() const;
    
    // Get resync status for peer
    std::optional<ResyncState> getResyncState(uint16_t peer_id) const;
    
    // Get stats
    const ResyncStats& getStats() const { return _stats; }
    
    // Callbacks
    using ResyncCallback = std::function<void(uint16_t peer_id, bool success, const std::string& message)>;
    void setResyncCallback(ResyncCallback cb) { _callback = cb; }

private:
    ConsensusEngine& _consensus;
    MeshManager& _mesh;
    Config _config;
    
    std::atomic<bool> _running{false};
    std::thread _resync_thread;
    std::mutex _mutex;
    std::unordered_map<uint16_t, ResyncState> _active_resyncs;
    uint32_t _next_resync_id = 1;
    
    ResyncStats _stats;
    std::mutex _stats_mutex;
    
    ResyncCallback _callback;
    
    // Resync processing
    void resyncLoop();
    
    // Process resync for peer
    bool processResync(uint16_t peer_id);
    
    // Request state from peer
    void requestState(uint16_t peer_id, uint32_t resync_id);
    
    // Send our state to peer
    void sendState(uint16_t peer_id, uint32_t resync_id);
    
    // Merge received state
    bool mergeState(uint16_t peer_id, const std::vector<uint8_t>& state_data);
    
    // Verify merged state
    bool verifyState(uint16_t peer_id);
    
    // Generate state hash
    std::vector<uint8_t> computeStateHash() const;
    
    // Handle timeout
    void handleTimeout(uint16_t peer_id);
    
    // Serialization
    std::vector<uint8_t> serializeState() const;
    bool deserializeState(const std::vector<uint8_t>& data);
    
    // Timing
    uint64_t nowUs() const;
    float elapsedMs(uint64_t start_us) const;
    
    // Callbacks
    std::function<void(uint16_t, bool, const std::string&)> _callback;
};

} // namespace swarm::comm

#endif // RESYNC_HPP