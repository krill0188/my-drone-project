#ifndef REALLOCATION_HPP
#define REALLOCATION_HPP

#include "cbba.hpp"
#include "task.hpp"
#include "mission.hpp"
#include "agent_state.hpp"
#include "swarm_map.hpp"
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace swarm::autonomy {

// Task reallocation handler for drone dropouts and dynamic changes
class ReallocationManager {
public:
    struct Config {
        float reallocation_timeout_s = 5.0f;      // Max time for reallocation (FR-006)
        float max_coverage_gap = 0.02f;           // Max coverage gap (SC-003)
        float dropout_detection_timeout_s = 2.0f; // Time to confirm dropout
        float reallocation_trigger_threshold = 0.1f; // Coverage gap threshold to trigger
        bool enable_predictive_reallocation = true; // Predict and pre-allocate
        int max_reallocation_attempts = 3;
    };
    
    struct ReallocationEvent {
        enum class Type {
            DRONE_DROPOUT,
            DRONE_JOIN,
            MISSION_CHANGE,
            COVERAGE_GAP,
            PRIORITY_CHANGE,
            EMERGENCY
        };
        
        Type type;
        uint16_t affected_drone_id = 0;
        std::vector<std::array<uint8_t, 16>> affected_tasks;
        uint64_t timestamp_us;
        std::string reason;
        float coverage_gap = 0.0f;
    };
    
    struct ReallocationStats {
        uint64_t total_reallocations = 0;
        uint64_t successful_reallocations = 0;
        uint64_t failed_reallocations = 0;
        float avg_reallocation_time_ms = 0.0f;
        float max_reallocation_time_ms = 0.0f;
        uint64_t drones_dropped = 0;
        uint64_t drones_joined = 0;
    };
    
    ReallocationManager(CbbaAllocator& allocator,
                        const std::function<std::vector<Task>()>& get_tasks,
                        const std::function<std::vector<uint16_t>()>& get_active_drones,
                        const std::function<AgentState(uint16_t)>& get_drone_state,
                        const std::function<SwarmMap*()>& get_swarm_map,
                        const Config& config = Config{});
    
    ~ReallocationManager();
    
    // Start/stop monitoring
    void start();
    void stop();
    
    // Trigger reallocation
    void triggerReallocation(const ReallocationEvent& event);
    
    // Handle specific events
    void onDroneDropout(uint16_t drone_id);
    void onDroneJoin(uint16_t drone_id);
    void onMissionChange(const Mission& new_mission);
    void onCoverageGapDetected(float gap_pct);
    
    // Get current status
    bool isReallocating() const;
    const ReallocationStats& getStats() const;
    
    // Callbacks
    using ReallocationCallback = std::function<void(bool success, const ReallocationEvent& event)>;
    void setReallocationCallback(ReallocationCallback cb) { _callback = cb; }

private:
    CbbaAllocator& _allocator;
    std::function<std::vector<Task>()> _get_tasks;
    std::function<std::vector<uint16_t>()> _get_active_drones;
    std::function<AgentState(uint16_t)> _get_drone_state;
    std::function<SwarmMap*()> _get_swarm_map;
    Config _config;
    
    std::atomic<bool> _running{false};
    std::atomic<bool> _reallocating{false};
    std::thread _monitor_thread;
    std::thread _reallocation_thread;
    std::mutex _event_mutex;
    std::condition_variable _event_cv;
    std::queue<ReallocationEvent> _event_queue;
    
    ReallocationStats _stats;
    ReallocationCallback _callback;
    
    // Monitor thread
    void monitorLoop();
    
    // Reallocation thread
    void reallocationLoop();
    
    // Process reallocation event
    bool processReallocation(const ReallocationEvent& event);
    
    // Specific handlers
    bool handleDroneDropout(uint16_t drone_id);
    bool handleDroneJoin(uint16_t drone_id);
    bool handleMissionChange(const Mission& mission);
    bool handleCoverageGap(float gap_pct);
    
    // Coverage calculation
    float calculateCoverageGap(const std::vector<Task>& tasks, 
                               const std::vector<uint16_t>& active_drones);
    
    // Gap detection
    bool detectCoverageGaps(float& gap_pct);
    
    // Emergency reallocation
    bool emergencyReallocate(const ReallocationEvent& event);
    
    // Timing
    uint64_t nowUs() const;
    float elapsedMs(uint64_t start_us) const;
    
    // Callbacks
    ReallocationCallback _callback;
};

} // namespace swarm::autonomy

#endif // REALLOCATION_HPP