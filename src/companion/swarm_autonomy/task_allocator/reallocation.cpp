#include "reallocation.hpp"
#include "cbba.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <chrono>
#include <algorithm>
#include <iostream>

namespace swarm::autonomy {

ReallocationManager::ReallocationManager(
    CbbaAllocator& allocator,
    const std::function<std::vector<Task>()>& get_tasks,
    const std::function<std::vector<uint16_t>()>& get_active_drones,
    const std::function<AgentState(uint16_t)>& get_drone_state,
    const std::function<SwarmMap*()>& get_swarm_map,
    const Config& config)
    : _allocator(allocator)
    , _get_tasks(get_tasks)
    , _get_active_drones(get_active_drones)
    , _get_drone_state(get_drone_state)
    , _get_swarm_map(get_swarm_map)
    , _config(config) {
}

ReallocationManager::~ReallocationManager() {
    stop();
}

void ReallocationManager::start() {
    if (_running.load()) return;
    
    _running.store(true);
    _monitor_thread = std::thread(&ReallocationManager::monitorLoop, this);
    _reallocation_thread = std::thread(&ReallocationManager::reallocationLoop, this);
}

void ReallocationManager::stop() {
    if (!_running.load()) return;
    
    _running.store(false);
    _event_cv.notify_all();
    
    if (_monitor_thread.joinable()) {
        _monitor_thread.join();
    }
    if (_reallocation_thread.joinable()) {
        _reallocation_thread.join();
    }
}

void ReallocationManager::triggerReallocation(const ReallocationEvent& event) {
    {
        std::lock_guard<std::mutex> lock(_event_mutex);
        _event_queue.push(event);
    }
    _event_cv.notify_one();
}

void ReallocationManager::onDroneDropout(uint16_t drone_id) {
    ReallocationEvent event;
    event.type = ReallocationEvent::Type::DRONE_DROPOUT;
    event.affected_drone_id = drone_id;
    event.timestamp_us = Timestamp::nowUs();
    event.reason = "Drone heartbeat lost";
    
    // Get tasks assigned to dropped drone
    auto tasks = _get_tasks();
    for (const auto& task : tasks) {
        if (task.assigned_drone && *task.assigned_drone == drone_id) {
            event.affected_tasks.push_back(task.id);
        }
    }
    
    triggerReallocation(event);
}

void ReallocationManager::onDroneJoin(uint16_t drone_id) {
    ReallocationEvent event;
    event.type = ReallocationEvent::Type::DRONE_JOIN;
    event.affected_drone_id = drone_id;
    event.timestamp_us = Timestamp::nowUs();
    event.reason = "New drone joined swarm";
    
    triggerReallocation(event);
}

void ReallocationManager::onMissionChange(const Mission& new_mission) {
    ReallocationEvent event;
    event.type = ReallocationEvent::Type::MISSION_CHANGE;
    event.timestamp_us = Timestamp::nowUs();
    event.reason = "Mission updated: " + new_mission.name;
    
    triggerReallocation(event);
}

void ReallocationManager::onCoverageGapDetected(float gap_pct) {
    if (gap_pct < _config.reallocation_trigger_threshold) return;
    
    ReallocationEvent event;
    event.type = ReallocationEvent::Type::COVERAGE_GAP;
    event.timestamp_us = Timestamp::nowUs();
    event.reason = "Coverage gap detected: " + std::to_string(gap_pct * 100) + "%";
    event.coverage_gap = gap_pct;
    
    triggerReallocation(event);
}

bool ReallocationManager::isReallocating() const {
    return _reallocating.load();
}

const ReallocationManager::ReallocationStats& ReallocationManager::getStats() const {
    return _stats;
}

void ReallocationManager::setReallocationCallback(ReallocationCallback cb) {
    _callback = cb;
}

void ReallocationManager::monitorLoop() {
    while (_running.load()) {
        // Check for coverage gaps
        float gap_pct = 0.0f;
        if (detectCoverageGaps(gap_pct)) {
            onCoverageGapDetected(gap_pct);
        }
        
        // Check for drone dropouts
        auto active_drones = _get_active_drones();
        // In real implementation, would compare with previous active drones
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void ReallocationManager::reallocationLoop() {
    while (_running.load()) {
        ReallocationEvent event;
        bool has_event = false;
        
        {
            std::unique_lock<std::mutex> lock(_event_mutex);
            _event_cv.wait_for(lock, std::chrono::seconds(1), [this] { 
                return !_event_queue.empty() || !_running.load(); 
            });
            
            if (!_running.load() && _event_queue.empty()) break;
            
            if (!_event_queue.empty()) {
                event = _event_queue.front();
                _event_queue.pop();
                has_event = true;
            }
        }
        
        if (!has_event) continue;
        
        _reallocating.store(true);
        
        auto start_time = Timestamp::nowUs();
        bool success = processReallocation(event);
        float elapsed_ms = (Timestamp::nowUs() - start_time) / 1000.0f;
        
        // Update stats
        {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.total_reallocations++;
            if (success) {
                _stats.successful_reallocations++;
            } else {
                _stats.failed_reallocations++;
            }
            _stats.avg_reallocation_time_ms = 
                0.9f * _stats.avg_reallocation_time_ms + 0.1f * elapsed_ms;
            _stats.max_reallocation_time_ms = 
                std::max(_stats.max_reallocation_time_ms, elapsed_ms);
            
            if (event.type == ReallocationEvent::Type::DRONE_DROPOUT) {
                _stats.drones_dropped++;
            } else if (event.type == ReallocationEvent::Type::DRONE_JOIN) {
                _stats.drones_joined++;
            }
        }
        
        // Callback
        if (_callback) {
            _callback(success, event);
        }
        
        _reallocating.store(false);
    }
}

bool ReallocationManager::processReallocation(const ReallocationEvent& event) {
    switch (event.type) {
        case ReallocationEvent::Type::DRONE_DROPOUT:
            return handleDroneDropout(event.affected_drone_id);
        case ReallocationEvent::Type::DRONE_JOIN:
            return handleDroneJoin(event.affected_drone_id);
        case ReallocationEvent::Type::MISSION_CHANGE:
            // Would need mission parameter
            return true;
        case ReallocationEvent::Type::COVERAGE_GAP:
            return handleCoverageGap(event.coverage_gap);
        default:
            return false;
    }
}

bool ReallocationManager::handleDroneDropout(uint16_t drone_id) {
    auto start = Timestamp::nowUs();
    
    // Get tasks assigned to dropped drone
    auto tasks = _get_tasks();
    std::vector<Task> affected_tasks;
    
    for (auto& task : tasks) {
        if (task.assigned_drone && *task.assigned_drone == drone_id) {
            // Mark task as needing reallocation
            task.status = TaskStatus::REEVALUATING;
            task.assigned_drone = std::nullopt;
            affected_tasks.push_back(task);
        }
    }
    
    if (affected_tasks.empty()) {
        return true;  // Nothing to reallocate
    }
    
    // Trigger CBBA reallocation
    _allocator.handleDroneDropout(drone_id);
    
    // Wait for reallocation to complete (with timeout)
    auto start = Timestamp::nowUs();
    while (Timestamp::nowUs() - start < _config.reallocation_timeout_s * 1'000'000) {
        bool all_reallocated = true;
        auto current_tasks = _get_tasks();
        
        for (const auto& task : affected_tasks) {
            auto it = std::find_if(current_tasks.begin(), current_tasks.end(),
                [&](const Task& t) { return t.id == task.id; });
            
            if (it == current_tasks.end() || !it->assigned_drone || *it->assigned_drone != drone_id) {
                // Task reallocated or still unallocated
                continue;
            }
            
            // Check if newly assigned
            if (it->assigned_drone && *it->assigned_drone != drone_id) {
                continue;  // Successfully reallocated
            }
            
            all_reallocated = false;
            break;
        }
        
        if (all_reallocated) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Verify coverage gap
    float gap = calculateCoverageGap(_get_tasks(), _get_active_drones());
    
    return gap <= _config.max_coverage_gap;
}

bool ReallocationManager::handleDroneJoin(uint16_t drone_id) {
    // New drone joined - trigger reallocation to utilize new resource
    _allocator.reevaluateTasks(_get_tasks(), _get_drone_state(drone_id), _get_swarm_map());
    
    // Verify coverage improved
    float gap = calculateCoverageGap(_get_tasks(), _get_active_drones());
    return gap <= _config.max_coverage_gap;
}

bool ReallocationManager::handleMissionChange(const Mission& mission) {
    // Full reallocation for new mission
    // Would need to rebuild task list and reallocate
    return true;
}

bool ReallocationManager::handleCoverageGap(float gap_pct) {
    // Trigger reallocation to close coverage gap
    _allocator.reevaluateTasks(_get_tasks(), _get_drone_state(0), _get_swarm_map());
    
    // Wait for reallocation
    auto start = Timestamp::nowUs();
    while (Timestamp::nowUs() - start < _config.reallocation_timeout_s * 1'000'000) {
        float new_gap = calculateCoverageGap(_get_tasks(), _get_active_drones());
        if (new_gap <= _config.max_coverage_gap) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return false;
}

float ReallocationManager::calculateCoverageGap(const std::vector<Task>& tasks,
                                                 const std::vector<uint16_t>& active_drones) {
    // Simplified coverage calculation
    int allocated = 0;
    int total = 0;
    
    for (const auto& task : tasks) {
        if (task.type == TaskType::COVER_AREA) {
            total++;
            if (task.assigned_drone && 
                std::find(active_drones.begin(), active_drones.end(), *task.assigned_drone) != active_drones.end()) {
                allocated++;
            }
        }
    }
    
    if (total == 0) return 0.0f;
    return 1.0f - static_cast<float>(allocated) / total;
}

bool ReallocationManager::detectCoverageGaps(float& gap_pct) {
    auto tasks = _get_tasks();
    auto active_drones = _get_active_drones();
    
    gap_pct = calculateCoverageGap(tasks, active_drones);
    return gap_pct > _config.reallocation_trigger_threshold;
}

bool ReallocationManager::emergencyReallocate(const ReallocationEvent& event) {
    // Fast-track reallocation for emergencies
    // Bypass normal timeouts
    return processReallocation(event);
}

uint64_t ReallocationManager::nowUs() const {
    return Timestamp::nowUs();
}

float ReallocationManager::elapsedMs(uint64_t start_us) const {
    return (Timestamp::nowUs() - start_us) / 1000.0f;
}

} // namespace swarm::autonomy