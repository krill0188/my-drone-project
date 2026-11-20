#ifndef AUCTION_LOOP_HPP
#define AUCTION_LOOP_HPP

#include "cbba.hpp"
#include "task.hpp"
#include "mission.hpp"
#include "agent_state.hpp"
#include "swarm_map.hpp"
#include <chrono>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>
#include <condition_variable>

namespace swarm::autonomy {

// Decentralized auction loop running at configurable rate (default 1 Hz)
class AuctionLoop {
public:
    struct Config {
        float rate_hz = 1.0f;                    // Auction frequency
        float bundle_build_timeout_s = 2.0f;     // Max time for bundle building
        float consensus_timeout_s = 3.0f;        // Max time for consensus
        float bid_timeout_s = 5.0f;              // Max time for bid collection
        int max_consecutive_failures = 3;        // Max failures before emergency
        bool enable_async = true;                // Asynchronous execution
        bool log_auction_events = true;          // Log auction events
    };
    
    enum class AuctionState {
        IDLE,
        BUNDLE_BUILDING,
        CONSENSUS,
        CONVERGED,
        FAILED,
        EMERGENCY
    };
    
    struct AuctionStats {
        uint64_t total_auctions = 0;
        uint64_t successful_auctions = 0;
        uint64_t failed_auctions = 0;
        float avg_bundle_build_time_ms = 0.0f;
        float avg_consensus_time_ms = 0.0f;
        float avg_convergence_iterations = 0.0f;
        uint64_t total_conflicts_resolved = 0;
        AuctionState current_state = AuctionState::IDLE;
        uint64_t last_auction_time_us = 0;
    };
    
    AuctionLoop(CbbaAllocator& allocator, 
                const std::function<std::vector<Task>()>& get_available_tasks,
                const std::function<AgentState()>& get_own_state,
                const std::function<SwarmMap*()>& get_swarm_map,
                const Config& config = Config{});
    
    ~AuctionLoop();
    
    // Start/stop the auction loop
    void start();
    void stop();
    
    // Trigger immediate auction (e.g., on mission change, drone dropout)
    void triggerAuction(const std::string& reason = "manual");
    
    // Get current state
    AuctionState getState() const { return _state; }
    const AuctionStats& getStats() const { return _stats; }
    
    // Callbacks
    using AuctionCallback = std::function<void(bool success, const std::vector<std::array<uint8_t, 16>>& allocated_tasks)>;
    void setAuctionCallback(AuctionCallback cb) { _auction_cb = cb; }
    
    // Emergency stop
    void emergencyStop(const std::string& reason);

private:
    CbbaAllocator& _allocator;
    std::function<std::vector<Task>()> _get_available_tasks;
    std::function<AgentState()> _get_own_state;
    std::function<SwarmMap*()> _get_swarm_map;
    Config _config;
    
    std::atomic<bool> _running{false};
    std::atomic<bool> _auction_triggered{false};
    std::string _trigger_reason;
    std::thread _auction_thread;
    std::mutex _trigger_mutex;
    std::condition_variable _trigger_cv;
    
    AuctionState _state = AuctionState::IDLE;
    AuctionStats _stats;
    std::mutex _stats_mutex;
    
    std::function<void(bool, const std::vector<std::array<uint8_t, 16>>&)> _auction_cb;
    
    // Main auction loop
    void auctionLoop();
    
    // Single auction cycle
    bool runAuctionCycle(const std::string& trigger_reason);
    
    // Phase 1: Bundle building
    bool buildBundlePhase(const std::vector<Task>& available_tasks, 
                          const AgentState& own_state,
                          const SwarmMap* map);
    
    // Phase 2: Consensus
    bool consensusPhase();
    
    // Check if auction should continue
    bool shouldContinueAuction() const;
    
    // Handle auction completion
    void handleAuctionCompletion(bool success);
    
    // Update statistics
    void updateStats(bool success, float bundle_time_ms, float consensus_time_ms, int iterations);
    
    // Check for emergency conditions
    bool checkEmergencyConditions(const AgentState& state) const;
    
    // Emergency handling
    void handleEmergency(const std::string& reason);
    
    // Timing utilities
    uint64_t nowUs() const;
    float elapsedMs(uint64_t start_us) const;
};

} // namespace swarm::autonomy

#endif // AUCTION_LOOP_HPP