#ifndef LANDING_SEQUENCE_HPP
#define LANDING_SEQUENCE_HPP

#include "swarm_formation.hpp"
#include <vector>
#include <array>
#include <functional>

namespace swarm::formation {

// Sequenced RTL and landing at designated pads
class LandingSequence {
public:
    struct Config {
        float rtl_altitude_m = 50.0f;           // RTL altitude
        float descent_speed_mps = 1.5f;         // Descent speed
        float landing_speed_mps = 0.5f;         // Final landing speed
        float hover_before_land_s = 2.0f;       // Hover time before landing
        float pad_tolerance_m = 0.5f;           // Landing pad tolerance
        float separation_min_m = 3.0f;          // Min separation between landing drones
        bool leader_lands_first = true;         // Leader lands first
        float leader_follower_delay_s = 3.0f;   // Delay between leader and follower landing
        bool use_precision_landing = true;      // Use precision landing (camera/IR)
        float precision_landing_altitude_m = 5.0f;  // Altitude to start precision landing
    };
    
    enum class Phase {
        IDLE,
        RTL_INITIATED,
        RTL_CLIMB,
        RTL_TRANSIT,
        RTL_HOVER,
        PRECISION_LANDING,
        FINAL_DESCENT,
        TOUCHDOWN,
        MOTOR_DISARM,
        COMPLETE,
        ABORTED
    };
    
    struct LandingPad {
        uint16_t drone_id;
        Vector3 position_enu;       // ENU coordinates
        float heading_deg = 0.0f;   // Preferred heading
        bool has_precision_target = false;  // IR marker, AprilTag, etc.
    };
    
    struct LandingState {
        Phase current_phase = Phase::IDLE;
        uint64_t phase_start_us = 0;
        uint16_t leader_id = 0;
        std::vector<LandingPad> landing_pads;
        std::vector<uint16_t> drones_landed;
        std::vector<uint16_t> motors_disarmed;
        bool leader_landed = false;
        std::string abort_reason;
    };
    
    LandingSequence(FormationController& controller, MeshManager& mesh, 
                    const std::vector<LandingPad>& pads, const Config& config = Config());
    
    // Start landing sequence (can be called for RTL or direct landing)
    bool start();
    
    // Update function (call at 50 Hz)
    bool update(float dt);
    
    // Abort landing sequence
    void abort(const std::string& reason);
    
    // Get current state
    const LandingState& getState() const { return _state; }
    Phase getCurrentPhase() const { return _state.current_phase; }
    bool isComplete() const { return _state.current_phase == Phase::COMPLETE; }
    bool isAborted() const { return _state.current_phase == Phase::ABORTED; }
    const std::string& getAbortReason() const { return _state.abort_reason; }
    
    // Callbacks
    using CompletionCallback = std::function<void(bool success, const std::string& message)>;
    void setCompletionCallback(CompletionCallback cb) { _completion_cb = cb; }
    
    // Emergency stop all motors immediately
    void emergencyStop();

private:
    FormationController& _controller;
    MeshManager& _mesh;
    Config _config;
    std::vector<LandingPad> _landing_pads;
    LandingState _state;
    
    std::function<void(bool, const std::string&)> _completion_cb;
    
    // Phase handlers
    bool handleRtlInitiated(float dt);
    bool handleRtlClimb(float dt);
    bool handleRtlTransit(float dt);
    bool handleRtlHover(float dt);
    bool handlePrecisionLanding(float dt);
    bool handleFinalDescent(float dt);
    bool handleTouchdown(float dt);
    bool handleMotorDisarm(float dt);
    
    // Helper functions
    bool assignLandingPads();
    bool commandRtl(uint16_t sysid);
    bool commandLand(uint16_t sysid);
    bool commandPrecisionLand(uint16_t sysid, const Vector3& pad_position);
    bool waitForPosition(uint16_t sysid, const Vector3& target, float tolerance);
    bool waitForAltitude(uint16_t sysid, float target_alt, float tolerance);
    bool checkPrecisionLandingReady(uint16_t sysid);
    bool checkTouchdown(uint16_t sysid);
    void disarmMotor(uint16_t sysid);
    bool checkSeparation(const std::vector<uint16_t>& landed_ids);
    
    // Time utilities
    float getPhaseElapsedS() const;
    bool isPhaseTimeout(float timeout_s) const;
    
    // Safety checks
    bool checkBatteryForLanding();
    bool checkWeatherForLanding();
    void advancePhase();
};

} // namespace swarm::formation

#endif // LANDING_SEQUENCE_HPP