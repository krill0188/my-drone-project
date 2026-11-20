#include "auction_loop.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <chrono>
#include <iostream>
#include <sstream>

namespace swarm::autonomy {

AuctionLoop::AuctionLoop(CbbaAllocator& allocator,
                         const std::function<std::vector<Task>()>& get_available_tasks,
                         const std::function<AgentState()>& get_own_state,
                         const std::function<SwarmMap*()>& get_swarm_map,
                         const Config& config)
    : _allocator(allocator)
    , _get_available_tasks(get_available_tasks)
    , _get_own_state(get_own_state)
    , _get_swarm_map(get_swarm_map)
    , _config(config) {
}

AuctionLoop::~AuctionLoop() {
    stop();
}

void AuctionLoop::start() {
    if (_running.load()) return;
    
    _running.store(true);
    _state = AuctionState::IDLE;
    _auction_thread = std::thread(&AuctionLoop::auctionLoop, this);
}

void AuctionLoop::stop() {
    if (!_running.load()) return;
    
    _running.store(false);
    _auction_triggered.store(true);
    _trigger_cv.notify_all();
    
    if (_auction_thread.joinable()) {
        _auction_thread.join();
    }
}

void AuctionLoop::triggerAuction(const std::string& reason) {
    std::lock_guard<std::mutex> lock(_trigger_mutex);
    _trigger_reason = reason;
    _auction_triggered.store(true);
    _trigger_cv.notify_one();
}

void AuctionLoop::emergencyStop(const std::string& reason) {
    handleEmergency(reason);
    stop();
}

void AuctionLoop::auctionLoop() {
    using namespace std::chrono;
    
    // Calculate period from rate
    auto period = microseconds(static_cast<long long>(1'000'000 / _config.rate_hz));
    
    while (_running.load()) {
        auto cycle_start = steady_clock::now();
        
        // Wait for trigger or periodic auction
        std::unique_lock<std::mutex> lock(_trigger_mutex);
        bool triggered = _auction_triggered.load();
        std::string reason = _trigger_reason;
        
        if (!triggered) {
            // Wait for trigger or timeout for periodic auction
            _trigger_cv.wait_for(lock, period, [this] { return _auction_triggered.load() || !_running.load(); });
            triggered = _auction_triggered.load();
            if (triggered) {
                reason = _trigger_reason;
            }
        }
        
        if (!_running.load()) break;
        
        if (triggered) {
            _auction_triggered.store(false);
            _trigger_reason.clear();
        }
        
        lock.unlock();
        
        // Run auction cycle
        bool is_periodic = reason.empty() || reason == "periodic";
        if (!is_periodic) {
            reason = _trigger_reason;
        } else {
            reason = "periodic";
        }
        
        bool success = runAuctionCycle(reason);
        
        // Handle auction completion
        handleAuctionCompletion(success);
        
        // Maintain rate
        auto cycle_end = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(cycle_end - cycle_start);
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

bool AuctionLoop::runAuctionCycle(const std::string& trigger_reason) {
    auto cycle_start = Timestamp::nowUs();
    
    // Check emergency conditions
    AgentState own_state = _get_own_state();
    if (checkEmergencyConditions(own_state)) {
        handleEmergency("Emergency condition detected");
        return false;
    }
    
    // Phase 1: Bundle building
    auto bundle_start = Timestamp::nowUs();
    _state = AuctionState::BUNDLE_BUILDING;
    
    auto available_tasks = _get_available_tasks();
    AgentState own_state = _get_own_state();
    SwarmMap* map = _get_swarm_map();
    
    bool bundle_success = buildBundlePhase(available_tasks, own_state, map);
    
    if (!bundle_success) {
        return false;
    }
    
    float bundle_time_ms = (Timestamp::nowUs() - bundle_start) / 1000.0f;
    
    // Phase 2: Consensus
    auto consensus_start = Timestamp::nowUs();
    _state = AuctionState::CONSENSUS;
    
    bool consensus_success = consensusPhase();
    float consensus_time_ms = (Timestamp::nowUs() - consensus_start) / 1000.0f;
    
    if (!consensus_success) {
        return false;
    }
    
    // Check convergence
    _state = AuctionState::CONVERGED;
    
    int iterations = _allocator.getState().iteration;
    handleAuctionCompletion(true);
    
    // Update stats
    float total_time_ms = (Timestamp::nowUs() - cycle_start) / 1000.0f;
    updateStats(true, bundle_time_ms, consensus_time_ms, _allocator.getState().iteration);
    
    return true;
}

bool AuctionLoop::buildBundlePhase(const std::vector<Task>& available_tasks,
                                   const AgentState& own_state,
                                   const SwarmMap* map) {
    try {
        _allocator.buildBundle(available_tasks, own_state, map);
        return true;
    } catch (const std::exception& e) {
        if (_config.log_auction_events) {
            std::cerr << "[AuctionLoop] Bundle building failed: " << e.what() << std::endl;
        }
        return false;
    }
}

bool AuctionLoop::consensusPhase() {
    try {
        _allocator.communicateBundles();
        
        // Wait for convergence with timeout
        auto start = Timestamp::nowUs();
        float timeout_ms = _config.consensus_timeout_s * 1000.0f;
        
        while (true) {
            if (_allocator.checkConvergence()) {
                return true;
            }
            
            float elapsed = (Timestamp::nowUs() - start) / 1000.0f;
            if (elapsed > timeout_ms) {
                if (_config.log_auction_events) {
                    std::cerr << "[AuctionLoop] Consensus timeout after " << elapsed << "ms" << std::endl;
                }
                return false;
            }
            
            // Small sleep to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception& e) {
        if (_config.log_auction_events) {
            std::cerr << "[AuctionLoop] Consensus failed: " << e.what() << std::endl;
        }
        return false;
    }
}

bool AuctionLoop::shouldContinueAuction() const {
    return _running.load() && _state != AuctionState::FAILED && _state != AuctionState::EMERGENCY;
}

void AuctionLoop::handleAuctionCompletion(bool success) {
    std::lock_guard<std::mutex> lock(_stats_mutex);
    
    if (success) {
        _state = AuctionState::CONVERGED;
        _stats.successful_auctions++;
    } else {
        _state = AuctionState::FAILED;
        _stats.failed_auctions++;
    }
    
    _stats.total_auctions++;
    _stats.current_state = _state;
    _stats.last_auction_time_us = Timestamp::nowUs();
    
    // Get allocated tasks
    auto allocated_tasks = _allocator.getAllocatedTasks();
    
    // Call callback
    if (_auction_cb) {
        _auction_cb(success, allocated_tasks);
    }
}

void AuctionLoop::updateStats(bool success, float bundle_time_ms, float consensus_time_ms, int iterations) {
    std::lock_guard<std::mutex> lock(_stats_mutex);
    
    // Exponential moving average for timing
    float alpha = 0.1f;
    _stats.avg_bundle_build_time_ms = alpha * bundle_time_ms + (1 - alpha) * _stats.avg_bundle_build_time_ms;
    _stats.avg_consensus_time_ms = alpha * consensus_time_ms + (1 - alpha) * _stats.avg_consensus_time_ms;
    _stats.avg_convergence_iterations = alpha * iterations + (1 - alpha) * _stats.avg_convergence_iterations;
}

bool AuctionLoop::checkEmergencyConditions(const AgentState& state) const {
    // Battery critical
    if (state.battery_pct < 0.15f) return true;
    
    // Health critical
    if (state.health >= 3) return true;  // CRITICAL or EMERGENCY
    
    // GPS lost in critical phase
    if (!state.gps_fix && _state != AuctionState::IDLE) return true;
    
    // Communication lost
    // Would check mesh QoS here
    
    return false;
}

void AuctionLoop::handleEmergency(const std::string& reason) {
    _state = AuctionState::EMERGENCY;
    
    if (_config.log_auction_events) {
        std::cerr << "[AuctionLoop] EMERGENCY: " << reason << std::endl;
    }
    
    // Trigger emergency callback
    if (_auction_cb) {
        _auction_cb(false, {});
    }
}

uint64_t AuctionLoop::nowUs() const {
    return Timestamp::nowUs();
}

float AuctionLoop::elapsedMs(uint64_t start_us) const {
    return (Timestamp::nowUs() - start_us) / 1000.0f;
}

void AuctionLoop::setAuctionCallback(AuctionCallback cb) {
    _auction_cb = cb;
}

} // namespace swarm::autonomy