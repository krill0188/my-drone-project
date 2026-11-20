#ifndef TAKEOFF_SEQUENCE_HPP
#define TAKEOFF_SEQUENCE_HPP

#include "swarm_formation.hpp"
#include <vector>
#include <array>
#include <functional>

namespace swarm::formation {

// Coordinated takeoff sequence for formation acquisition
class TakeoffSequence {
public:
    struct Config {
        float takeoff_altitude_m = 10.0f;       // Target altitude
        float takeoff_speed_mps = 2.0f;         // Vertical speed
        float formation_acquisition_timeout_s = 10.0f;  // Max time to acquire formation
        float hold_altitude_before_formation_s = 2.0f;  // Hover at altitude before moving to slots
        float slot_approach_speed_mps = 3.0f;   // Speed to approach formation slots
        float position_tolerance_m = 0.5f;      // Formation position tolerance
        bool leader_takes_off_first = true;     // Leader takes off before followers
        float leader_follower_delay_s = 1.0f;   // Delay between leader and follower takeoff
    };
    
    enum class Phase {
        IDLE,
        PREARM_CHECKS,
        ARMING,
        LEADER_TAKEOFF,
        FOLLOWER_TAKEOFF,
        ALTITUDE_HOLD,
        FORMATION_ACQUISITION,
        FORMATION_HOLD,
        COMPLETE,
        ABORTED
    };
    
    struct TakeoffState {
        Phase current_phase = Phase::IDLE;
        uint64_t phase_start_us = 0;
        bool leader_airborne = false;
        std::vector<uint16_t> followers_airborne;
        Vector3 leader_takeoff_pos;
        std::vector<Vector3> follower_takeoff_positions;
        bool formation_acquired = false;
        std::string abort_reason;
    };
    
    TakeoffSequence(FormationController& controller, MeshManager& mesh, const Config& config = Config());
    
    // Start coordinated takeoff sequence
    bool start();
    
    // Update function (call at 50 Hz)
    bool update(float dt);
    
    // Abort takeoff sequence
    void abort(const std::string& reason);
    
    // Get current state
    const TakeoffState& getState() const { return _state; }
    Phase getCurrentPhase() const { return _state.current_phase; }
    bool isComplete() const { return _state.current_phase == Phase::COMPLETE; }
    bool isAborted() const { return _state.current_phase == Phase::ABORTED; }
    const std::string& getAbortReason() const { return _state.abort_reason; }
    
    // Callbacks
    using CompletionCallback = std::function<void(bool success, const std::string& message)>;
    void setCompletionCallback(CompletionCallback cb) { _completion_cb = cb; }

private:
    FormationController& _controller;
    MeshManager& _mesh;
    Config _config;
    TakeoffState _state;
    
    std::function<void(bool, const std::string&)> _completion_cb;
    
    // Phase handlers
    bool handlePrearmChecks(float dt);
    bool handleArming(float dt);
    bool handleLeaderTakeoff(float dt);
    bool handleFollowerTakeoff(float dt);
    bool handleAltitudeHold(float dt);
    bool handleFormationAcquisition(float dt);
    bool handleFormationHold(float dt);
    
    // Helper functions
    bool performPrearmChecks();
    bool armAllDrones();
    bool disarmAllDrones();
    bool commandTakeoff(uint16_t sysid, float altitude);
    bool waitForAltitude(uint16_t sysid, float target_alt, float tolerance);
    bool moveToFormationSlot(uint16_t follower_id, const Vector3& target_offset_flu);
    bool checkFormationAcquired();
    
    // Time utilities
    float getPhaseElapsedS() const;
    bool isPhaseTimeout(float timeout_s) const;
    
    // Safety checks
    bool checkBatterySufficient();
    bool checkGPSFix();
    bool checkWeatherConditions();
    bool checkGeofenceClearance();
};

} // namespace swarm::formation

#endif // TAKEOFF_SEQUENCE_HPP