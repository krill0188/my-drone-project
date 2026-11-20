#ifndef CBBA_HPP
#define CBBA_HPP

#include <vector>
#include <array>
#include <unordered_map>
#include <optional>
#include <functional>
#include <mutex>
#include "task.hpp"
#include "mission.hpp"
#include "capability.hpp"
#include "swarm_map.hpp"
#include "swarm_comm/mesh_manager.hpp"

namespace swarm::autonomy {

// CBBA (Consensus-Based Bundle Algorithm) for decentralized task allocation
class CbbaAllocator {
public:
    struct Config {
        float auction_rate_hz = 1.0f;         // CBBA iteration rate
        float bid_timeout_s = 5.0f;           // Max time for bid collection
        float consensus_threshold = 0.66f;    // 2/3 majority for convergence
        int max_iterations = 50;              // Max CBBA iterations
        float convergence_epsilon = 0.01f;    // Cost change threshold
        bool enable_async = true;             // Asynchronous consensus
    };
    
    struct Bid {
        uint16_t drone_id;
        std::array<uint8_t, 16> task_id;      // UUID
        float cost;
        CapabilityBitmap capabilities;
        uint32_t sequence;
        uint64_t timestamp_us;
    };
    
    struct Bundle {
        std::vector<std::array<uint8_t, 16>> tasks;    // Task IDs in bundle
        std::vector<float> costs;                      // Bid costs for each task
        std::vector<uint32_t> timestamps;              // When each task was added
        uint32_t iteration = 0;
        bool converged = false;
    };
    
    struct CBBAState {
        Bundle bundle;
        std::unordered_map<uint16_t, Bundle> neighbor_bundles;
        uint32_t iteration = 0;
        bool converged = false;
        uint64_t last_update_us = 0;
    };
    
    CbbaAllocator(uint16_t own_id, const CapabilityBitmap& capabilities, 
                  const Config& config = Config{});
    
    // Main CBBA functions
    void init();
    
    // Phase 1: Bundle building (local)
    void buildBundle(const std::vector<Task>& available_tasks,
                     const AgentState& own_state,
                     const SwarmMap* map = nullptr);
    
    // Phase 2: Consensus (communicate with neighbors)
    void communicateBundles();
    
    // Process incoming bid from neighbor
    void processBid(const Bid& bid);
    
    // Process incoming bundle from neighbor
    void processNeighborBundle(uint16_t neighbor_id, const Bundle& bundle);
    
    // Check convergence
    bool checkConvergence();
    
    // Get final allocation
    const CBBAState& getState() const { return _state; }
    const std::vector<std::array<uint8_t, 16>>& getAllocatedTasks() const { return _state.bundle.tasks; }
    const std::vector<float>& getAllocatedCosts() const { return _state.bundle.costs; }
    
    // Handle drone dropout
    void handleDroneDropout(uint16_t dropped_drone_id);
    
    // Re-evaluate tasks
    void reevaluateTasks(const std::vector<Task>& all_tasks,
                         const AgentState& own_state,
                         const SwarmMap* map = nullptr);
    
    // Get bundle for transmission
    const Bundle& getBundle() const { return _state.bundle; }
    
    // Message handlers for MAVLink/ROS 2
    void handleTaskBidMessage(uint16_t sender_id, const std::vector<uint8_t>& data);
    void handleTaskAwardMessage(uint16_t sender_id, const std::vector<uint8_t>& data);
    
    // Serialization
    std::vector<uint8_t> serializeBundle() const;
    static std::optional<Bundle> deserializeBundle(const std::vector<uint8_t>& data);
    std::vector<uint8_t> serializeBid(const Bid& bid) const;
    static std::optional<Bid> deserializeBid(const std::vector<uint8_t>& data);

private:
    uint16_t _own_id;
    CapabilityBitmap _capabilities;
    Config _config;
    CBBAState _state;
    
    std::mutex _mutex;
    swarm::comm::MeshManager* _mesh = nullptr;
    
    // CBBA core functions
    void phase1_BundleBuilding(const std::vector<Task>& available_tasks,
                               const AgentState& own_state,
                               const SwarmMap* map);
    
    void phase2_Consensus();
    
    float computeBidCost(const Task& task, const AgentState& state, const SwarmMap* map);
    void addTaskToBundle(const std::array<uint8_t, 16>& task_id, float cost);
    void removeTaskFromBundle(const std::array<uint8_t, 16>& task_id);
    
    // Consensus functions
    void sendBundleToNeighbors();
    void receiveNeighborBundles();
    bool resolveConflicts();
    bool checkConvergenceInternal();
    
    // Conflict resolution (CBBA rule: highest bidder wins)
    void resolveTaskConflict(const std::array<uint8_t, 16>& task_id);
    
    // Handle drone dropout
    void reallocateDroppedTasks(uint16_t dropped_id);
    
    // Helper functions
    bool isTaskInBundle(const std::array<uint8_t, 16>& task_id) const;
    float getTaskCostInBundle(const std::array<uint8_t, 16>& task_id) const;
    void updateBundleTimestamp(const std::array<uint8_t, 16>& task_id);
    
    // Communication
    void broadcastBundle();
    void sendBidToNeighbor(uint16_t neighbor_id, const std::array<uint8_t, 16>& task_id, float cost);
};

} // namespace swarm::autonomy

#endif // CBBA_HPP