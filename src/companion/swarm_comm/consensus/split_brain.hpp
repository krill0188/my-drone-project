#ifndef SPLIT_BRAIN_HPP
#define SPLIT_BRAIN_HPP

#include "consensus.hpp"
#include "swarm_comm/mesh_manager.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>

namespace swarm::comm {

// Split-brain detection and recovery for mesh partitions
class SplitBrainDetector {
public:
    struct Config {
        float partition_detection_timeout_s = 10.0f;  // Time to detect partition
        float merge_confirmation_timeout_s = 5.0f;    // Time to confirm merge
        int min_quorum_size = 3;                      // Min nodes for quorum
        bool auto_reconcile = true;                   // Auto-reconcile on merge
        float state_divergence_threshold = 0.1f;      // Max allowed divergence
    };
    
    struct PartitionInfo {
        std::vector<uint16_t> nodes;           // Nodes in this partition
        uint64_t detected_time_us;             // When partition detected
        std::vector<uint8_t> state_hash;       // State hash of partition
        int node_count = 0;
        bool has_quorum = false;
    };
    
    struct MergeEvent {
        uint64_t timestamp_us;
        std::vector<uint16_t> partition_a_nodes;
        std::vector<uint16_t> partition_b_nodes;
        bool reconciled = false;
        std::string reconciliation_method;
    };
    
    struct Stats {
        uint64_t partitions_detected = 0;
        uint64_t merges_completed = 0;
        uint64_t reconciliations_failed = 0;
        uint64_t state_divergences_detected = 0;
    };
    
    SplitBrainDetector(ConsensusEngine& consensus, MeshManager& mesh, 
                       const Config& config = Config{});
    
    ~SplitBrainDetector();
    
    // Start/stop detection
    void start();
    void stop();
    
    // Check for partitions
    void checkForPartitions();
    
    // Handle partition merge
    void handleMerge(const std::vector<uint16_t>& nodes_a, const std::vector<uint16_t>& nodes_b);
    
    // Force reconciliation
    bool forceReconciliation(const std::vector<uint16_t>& nodes);
    
    // Get current partitions
    std::vector<PartitionInfo> getPartitions() const;
    
    // Get merge history
    std::vector<MergeEvent> getMergeHistory() const;
    
    // Get stats
    const Stats& getStats() const { return _stats; }
    
    // Callbacks
    using PartitionCallback = std::function<void(const PartitionInfo& partition)>;
    void setPartitionCallback(PartitionCallback cb) { _partition_cb = cb; }
    
    using MergeCallback = std::function<void(const MergeEvent& merge)>;
    void setMergeCallback(MergeCallback cb) { _merge_cb = cb; }

private:
    ConsensusEngine& _consensus;
    MeshManager& _mesh;
    Config _config;
    
    std::atomic<bool> _running{false};
    std::thread _detector_thread;
    std::mutex _mutex;
    
    std::unordered_map<uint16_t, PartitionInfo> _partitions;  // representative -> partition
    std::vector<MergeEvent> _merge_history;
    
    Stats _stats;
    std::mutex _stats_mutex;
    
    std::function<void(const PartitionInfo&)> _partition_cb;
    std::function<void(const MergeEvent&)> _merge_cb;
    
    // Detection thread
    void detectorLoop();
    
    // Detect partitions using connectivity graph
    std::vector<std::vector<uint16_t>> detectPartitions();
    
    // Check if two partitions can merge
    bool canMerge(const PartitionInfo& a, const PartitionInfo& b);
    
    // Merge partitions
    bool mergePartitions(const PartitionInfo& a, const PartitionInfo& b);
    
    // Reconcile state between partitions
    bool reconcileState(const PartitionInfo& a, const PartitionInfo& b);
    
    // Compute state hash for partition
    std::vector<uint8_t> computePartitionHash(const std::vector<uint16_t>& nodes);
    
    // Check quorum
    bool hasQuorum(const PartitionInfo& partition) const;
    
    // Timing
    uint64_t nowUs() const;
    float elapsedS(uint64_t start_us) const;
    
    // Callbacks
    std::function<void(const PartitionInfo&)> _partition_cb;
    std::function<void(const MergeEvent&)> _merge_cb;
};

} // namespace swarm::comm

#endif // SPLIT_BRAIN_HPP