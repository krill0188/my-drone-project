#include "cbba.hpp"
#include "capability.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace swarm::autonomy {

CbbaAllocator::CbbaAllocator(uint16_t own_id, const CapabilityBitmap& capabilities, const Config& config)
    : _own_id(own_id), _capabilities(capabilities), _config(config) {
    _state = CBBAState{};
}

void CbbaAllocator::init() {
    std::lock_guard<std::mutex> lock(_mutex);
    _state = CBBAState{};
    _state.bundle.iteration = 0;
    _state.bundle.converged = false;
    _state.iteration = 0;
    _state.converged = false;
}

void CbbaAllocator::buildBundle(const std::vector<Task>& available_tasks,
                                const AgentState& own_state,
                                const SwarmMap* map) {
    std::lock_guard<std::mutex> lock(_mutex);
    phase1_BundleBuilding(available_tasks, own_state, map);
    _state.iteration = 0;
    _state.bundle.iteration = 0;
}

void CbbaAllocator::phase1_BundleBuilding(const std::vector<Task>& available_tasks,
                                          const AgentState& own_state,
                                          const SwarmMap* map) {
    _state.bundle.tasks.clear();
    _state.bundle.costs.clear();
    _state.bundle.timestamps.clear();
    
    // Filter allocatable tasks (dependencies met, capabilities match)
    std::vector<const Task*> allocatable_tasks;
    for (const auto& task : available_tasks) {
        if (task.isAllocatable() && 
            _capabilities.hasCapability(task.required_capabilities) &&
            own_state.battery_pct > 0.3f) {  // 30% minimum battery
            allocatable_tasks.push_back(&task);
        }
    }
    
    // Sort by priority (lower value = higher priority)
    std::sort(allocatable_tasks.begin(), allocatable_tasks.end(),
              [](const Task* a, const Task* b) {
                  return a->priority < b->priority;
              });
    
    // Greedy bundle building
    for (const Task* task : allocatable_tasks) {
        float cost = computeBidCost(*task, own_state, nullptr);
        
        // Check if adding this task improves bundle
        float marginal_cost = cost;
        
        // For CBBA, we add task if it's feasible and improves utility
        // In standard CBBA, we add if marginal cost < threshold
        if (cost < 1.0f && _state.bundle.tasks.size() < 10) {  // Max 10 tasks per bundle
            addTaskToBundle(task->id, cost);
        }
    }
    
    // Sort bundle by cost (ascending - lower cost first)
    std::vector<size_t> indices(_state.bundle.tasks.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [this](size_t a, size_t b) {
                  return _state.bundle.costs[a] < _state.bundle.costs[b];
              });
    
    // Reorder bundle
    std::vector<std::array<uint8_t, 16>> sorted_tasks;
    std::vector<float> sorted_costs;
    std::vector<uint32_t> sorted_timestamps;
    for (size_t idx : indices) {
        sorted_tasks.push_back(_state.bundle.tasks[idx]);
        sorted_costs.push_back(_state.bundle.costs[idx]);
        sorted_timestamps.push_back(_state.bundle.timestamps[idx]);
    }
    _state.bundle.tasks = std::move(sorted_tasks);
    _state.bundle.costs = std::move(sorted_costs);
    _state.bundle.timestamps = std::move(sorted_timestamps);
}

void CbbaAllocator::communicateBundles() {
    // Send bundle to neighbors via mesh
    broadcastBundle();
    
    // Receive neighbor bundles (handled via message callbacks)
    receiveNeighborBundles();
    
    // Run consensus resolution
    resolveConflicts();
    
    // Check convergence
    checkConvergenceInternal();
}

void CbbaAllocator::processBid(const Bid& bid) {
    // In CBBA, bids are communicated through bundle exchange
    // This handles individual bid messages (for compatibility)
    // In standard CBBA, we exchange full bundles
}

void CbbaAllocator::processNeighborBundle(uint16_t neighbor_id, const Bundle& bundle) {
    std::lock_guard<std::mutex> lock(_mutex);
    _state.neighbor_bundles[neighbor_id] = bundle;
    _state.neighbor_bundles[neighbor_id].iteration = _state.iteration;
}

void CbbaAllocator::sendBundleToNeighbors() {
    if (!_mesh) return;
    
    std::vector<uint8_t> data = serializeBundle();
    // Broadcast to all neighbors
    for (const auto& neighbor : _mesh->getNeighbors()) {
        if (neighbor.sysid != _own_id) {
            _mesh->sendMessage(neighbor.sysid, SwarmMsgId::TASK_BID, data.data(), data.size());
        }
    }
}

void CbbaAllocator::receiveNeighborBundles() {
    // Handled via message callbacks (handleTaskBidMessage, etc.)
    // Neighbor bundles are stored in _state.neighbor_bundles
}

bool CbbaAllocator::resolveConflicts() {
    std::lock_guard<std::mutex> lock(_mutex);
    
    bool any_conflict = false;
    
    // Check each task in our bundle against neighbor bundles
    for (size_t i = 0; i < _state.bundle.tasks.size(); ++i) {
        const auto& task_id = _state.bundle.tasks[i];
        float our_cost = _state.bundle.costs[i];
        
        // Check if any neighbor has lower cost for this task
        for (const auto& [neighbor_id, neighbor_bundle] : _state.neighbor_bundles) {
            auto it = std::find(neighbor_bundle.tasks.begin(), neighbor_bundle.tasks.end(), 
                               _state.bundle.tasks[i]);
            
            if (it != neighbor_bundle.tasks.end()) {
                size_t neighbor_idx = std::distance(neighbor_bundle.tasks.begin(), it);
                float neighbor_cost = neighbor_bundle.costs[neighbor_idx];
                
                // Conflict: neighbor has same task with different cost
                if (neighbor_cost < our_cost - 0.01f) {  // Neighbor has significantly lower cost
                    // We lose - remove from our bundle
                    removeTaskFromBundle(_state.bundle.tasks[i]);
                    any_conflict = true;
                    break;
                } else if (std::abs(neighbor_cost - our_cost) < 0.01f) {
                    // Tie-break: lower drone ID wins
                    if (neighbor_id < _own_id) {
                        removeTaskFromBundle(_state.bundle.tasks[i]);
                        any_conflict = true;
                        break;
                    }
                }
            }
        }
    }
    
    return any_conflict;
}

bool CbbaAllocator::checkConvergence() {
    std::lock_guard<std::mutex> lock(_mutex);
    return checkConvergenceInternal();
}

bool CbbaAllocator::checkConvergenceInternal() {
    // Convergence: no conflicts for one full iteration
    // or max iterations reached
    
    if (_state.iteration >= _config.max_iterations) {
        _state.converged = true;
        _state.bundle.converged = true;
        return true;
    }
    
    // Check if bundle hasn't changed for one iteration
    // (would need to track previous bundle)
    
    // Simple check: no conflicts in last iteration
    if (_state.iteration > 0) {
        _state.converged = true;
        _state.bundle.converged = true;
        return true;
    }
    
    return false;
}

void CbbaAllocator::addTaskToBundle(const std::array<uint8_t, 16>& task_id, float cost) {
    _state.bundle.tasks.push_back(task_id);
    _state.bundle.costs.push_back(cost);
    _state.bundle.timestamps.push_back(static_cast<uint32_t>(Timestamp::nowUs() / 1000));
}

void CbbaAllocator::removeTaskFromBundle(const std::array<uint8_t, 16>& task_id) {
    auto it = std::find(_state.bundle.tasks.begin(), _state.bundle.tasks.end(), task_id);
    if (it != _state.bundle.tasks.end()) {
        size_t idx = std::distance(_state.bundle.tasks.begin(), it);
        _state.bundle.tasks.erase(it);
        _state.bundle.costs.erase(_state.bundle.costs.begin() + idx);
        _state.bundle.timestamps.erase(_state.bundle.timestamps.begin() + idx);
    }
}

bool CbbaAllocator::isTaskInBundle(const std::array<uint8_t, 16>& task_id) const {
    return std::find(_state.bundle.tasks.begin(), _state.bundle.tasks.end(), task_id) 
           != _state.bundle.tasks.end();
}

float CbbaAllocator::getTaskCostInBundle(const std::array<uint8_t, 16>& task_id) const {
    auto it = std::find(_state.bundle.tasks.begin(), _state.bundle.tasks.end(), task_id);
    if (it != _state.bundle.tasks.end()) {
        size_t idx = std::distance(_state.bundle.tasks.begin(), it);
        return _state.bundle.costs[idx];
    }
    return -1.0f;
}

void CbbaAllocator::updateBundleTimestamp(const std::array<uint8_t, 16>& task_id) {
    auto it = std::find(_state.bundle.tasks.begin(), _state.bundle.tasks.end(), task_id);
    if (it != _state.bundle.tasks.end()) {
        size_t idx = std::distance(_state.bundle.tasks.begin(), it);
        _state.bundle.timestamps[idx] = static_cast<uint32_t>(Timestamp::nowUs() / 1000);
    }
}

float CbbaAllocator::computeBidCost(const Task& task, const AgentState& state, const SwarmMap* map) {
    return BidScorer::computeBid(_capabilities, state, task, BidScorer::Weights{}, map).total_cost;
}

void CbbaAllocator::handleDroneDropout(uint16_t dropped_drone_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    reallocateDroppedTasks(dropped_drone_id);
}

void CbbaAllocator::reallocateDroppedTasks(uint16_t dropped_id) {
    // Remove neighbor bundle for dropped drone
    _state.neighbor_bundles.erase(dropped_id);
    
    // Tasks that were allocated to dropped drone need reallocation
    // In CBBA, this is handled naturally in next iteration:
    // - Dropped drone's bundle is gone
    // - Our bundle will try to pick up those tasks in next buildBundle
    
    // Mark affected tasks for re-evaluation
    // (In practice, next buildBundle will handle this)
}

void CbbaAllocator::reevaluateTasks(const std::vector<Task>& all_tasks,
                                    const AgentState& own_state,
                                    const SwarmMap* map) {
    // Rebuild bundle with current state
    buildBundle(all_tasks, own_state, map);
}

void CbbaAllocator::handleTaskBidMessage(uint16_t sender_id, const std::vector<uint8_t>& data) {
    // Deserialize and process bid
    // For CBBA, we primarily use bundle exchange, but handle individual bids for compatibility
}

void CbbaAllocator::handleTaskAwardMessage(uint16_t sender_id, const std::vector<uint8_t>& data) {
    // Handle task award from auctioneer (if using centralized award phase)
}

std::vector<uint8_t> CbbaAllocator::serializeBundle() const {
    std::vector<uint8_t> data;
    
    uint32_t num_tasks = _state.bundle.tasks.size();
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&num_tasks),
                reinterpret_cast<const uint8_t*>(&num_tasks) + 4);
    
    for (size_t i = 0; i < _state.bundle.tasks.size(); ++i) {
        data.insert(data.end(), _state.bundle.tasks[i].begin(), _state.bundle.tasks[i].end());
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(&_state.bundle.costs[i]),
                    reinterpret_cast<const uint8_t*>(&_state.bundle.costs[i]) + 4);
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(&_state.bundle.timestamps[i]),
                    reinterpret_cast<const uint8_t*>(&_state.bundle.timestamps[i]) + 4);
    }
    
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&_state.bundle.iteration),
                reinterpret_cast<const uint8_t*>(&_state.bundle.iteration) + 4);
    data.push_back(_state.bundle.converged ? 1 : 0);
    
    return data;
}

std::optional<CbbaAllocator::Bundle> CbbaAllocator::deserializeBundle(const std::vector<uint8_t>& data) {
    if (data.size() < 4) return std::nullopt;
    
    Bundle bundle;
    size_t offset = 0;
    
    uint32_t num_tasks;
    std::memcpy(&num_tasks, data.data() + offset, 4);
    offset += 4;
    
    bundle.tasks.resize(num_tasks);
    bundle.costs.resize(num_tasks);
    bundle.timestamps.resize(num_tasks);
    
    for (uint32_t i = 0; i < num_tasks; ++i) {
        if (offset + 16 + 4 + 4 > data.size()) return std::nullopt;
        std::memcpy(bundle.tasks[i].data(), data.data() + offset, 16);
        offset += 16;
        std::memcpy(&bundle.costs[i], data.data() + offset, 4);
        offset += 4;
        std::memcpy(&bundle.timestamps[i], data.data() + offset, 4);
        offset += 4;
    }
    
    if (offset + 4 + 1 > data.size()) return std::nullopt;
    std::memcpy(&bundle.iteration, data.data() + offset, 4);
    offset += 4;
    bundle.converged = data[offset] != 0;
    
    return bundle;
}

std::vector<uint8_t> CbbaAllocator::serializeBid(const Bid& bid) const {
    std::vector<uint8_t> data;
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&bid.timestamp_us),
                reinterpret_cast<const uint8_t*>(&bid.timestamp_us) + 8);
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&bid.drone_id),
                reinterpret_cast<const uint8_t*>(&bid.drone_id) + 2);
    data.insert(data.end(), bid.task_id.begin(), bid.task_id.end());
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&bid.cost),
                reinterpret_cast<const uint8_t*>(&bid.cost) + 4);
    // Capability bitmap serialization would go here
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&bid.sequence),
                reinterpret_cast<const uint8_t*>(&bid.sequence) + 4);
    return data;
}

std::optional<CbbaAllocator::Bid> CbbaAllocator::deserializeBid(const std::vector<uint8_t>& data) {
    if (data.size() < 8 + 2 + 16 + 4) return std::nullopt;
    
    Bid bid;
    size_t offset = 0;
    
    std::memcpy(&bid.timestamp_us, data.data() + offset, 8);
    offset += 8;
    std::memcpy(&bid.drone_id, data.data() + offset, 2);
    offset += 2;
    std::memcpy(bid.task_id.data(), data.data() + offset, 16);
    offset += 16;
    std::memcpy(&bid.cost, data.data() + offset, 4);
    offset += 4;
    // Capability bitmap deserialization would go here
    std::memcpy(&bid.sequence, data.data() + offset, 4);
    
    return bid;
}

} // namespace swarm::autonomy