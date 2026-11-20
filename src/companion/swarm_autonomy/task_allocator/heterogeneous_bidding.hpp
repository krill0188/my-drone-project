#ifndef HETEROGENEOUS_BIDDING_HPP
#define HETEROGENEOUS_BIDDING_HPP

#include "capability.hpp"
#include "task.hpp"
#include "agent_state.hpp"
#include "swarm_map.hpp"
#include <vector>
#include <map>

namespace swarm::autonomy {

// Heterogeneous capability-aware bidding for CBBA
class HeterogeneousBidder {
public:
    struct Weights {
        float base_cost_weight = 1.0f;
        float capability_match_weight = 2.0f;
        float distance_weight = 1.5f;
        float energy_weight = 1.5f;
        float time_weight = 1.0f;
        float risk_weight = 1.5f;
        float heterogeneity_bonus_weight = 0.5f;  // Bonus for unique capabilities
    };
    
    struct DroneProfile {
        uint16_t drone_id;
        CapabilityBitmap capabilities;
        AgentState state;
        std::string platform_type;  // "quad", "hex", "vtol", "fixed_wing"
        float max_speed_mps = 15.0f;
        float hover_efficiency = 1.0f;  // Relative to quad
        float forward_efficiency = 1.0f;
    };
    
    struct BidEvaluation {
        float total_cost = 0.0f;
        float distance_cost = 0.0f;
        float capability_cost = 0.0f;
        float energy_cost = 0.0f;
        float time_cost = 0.0f;
        float risk_cost = 0.0f;
        float heterogeneity_bonus = 0.0f;
        bool feasible = true;
        std::string infeasible_reason;
        std::map<std::string, float> cost_breakdown;
    };
    
    HeterogeneousBidder(const Weights& weights = Weights{});
    
    // Evaluate bid for a drone-task pair
    BidEvaluation evaluateBid(const DroneProfile& drone,
                              const Task& task,
                              const AgentState& state,
                              const SwarmMap* map = nullptr,
                              const std::vector<DroneProfile>* all_drones = nullptr);
    
    // Compare bids from multiple drones for the same task
    struct ComparisonResult {
        uint16_t winner_id;
        float winning_cost;
        std::vector<std::pair<uint16_t, float>> all_bids;  // (drone_id, cost)
        bool tie = false;
    };
    
    ComparisonResult compareBids(const Task& task,
                                  const std::vector<DroneProfile>& drones,
                                  const std::vector<AgentState>& states,
                                  const SwarmMap* map = nullptr);
    
    // Adjust weights based on mission type
    void setMissionWeights(MissionType mission_type);
    
    // Get platform-specific efficiency factors
    static float getHoverEfficiency(const std::string& platform_type);
    static float getForwardEfficiency(const std::string& platform_type);
    static float getMaxSpeed(const std::string& platform_type);
    static float getEnduranceFactor(const std::string& platform_type);
    
    // Payload capacity check
    static bool canCarryPayload(const CapabilityBitmap& caps, float payload_kg);
    
    // Sensor suitability for task type
    static float sensorSuitability(const CapabilityBitmap& caps, TaskType task_type);
    
private:
    Weights _weights;
    
    // Cost components
    float computeDistanceCost(const DroneProfile& drone,
                              const AgentState& state,
                              const Task& task);
    
    float computeCapabilityCost(const DroneProfile& drone, const Task& task);
    float computeEnergyCost(const DroneProfile& drone,
                            const AgentState& state,
                            const Task& task,
                            const SwarmMap* map);
    float computeTimeCost(const DroneProfile& drone,
                          const AgentState& state,
                          const Task& task);
    float computeRiskCost(const DroneProfile& drone,
                          const AgentState& state,
                          const Task& task,
                          const SwarmMap* map);
    float computeHeterogeneityBonus(const DroneProfile& drone,
                                     const Task& task,
                                     const std::vector<DroneProfile>* all_drones);
    
    // Platform-specific adjustments
    float applyPlatformAdjustment(const DroneProfile& drone, float base_cost, TaskType task_type);
    
    // Normalization
    float normalize(float value, float min_val, float max_val);
};

} // namespace swarm::autonomy

#endif // HETEROGENEOUS_BIDDING_HPP