#ifndef CAPABILITY_HPP
#define CAPABILITY_HPP

#include <array>
#include <cstdint>
#include <vector>
#include <string>
#include "task.hpp"

namespace swarm::autonomy {

// Capability bitmap for drone capabilities
struct CapabilityBitmap {
    bool has_rtk_gps = false;
    bool has_stereo_camera = false;
    bool has_lidar_2d = false;
    bool has_lidar_3d = false;
    bool has_uwb = false;
    bool has_5g_modem = false;
    bool has_ai_compute = false;
    uint16_t max_flight_time_s = 0;
    float max_payload_kg = 0.0f;
    uint8_t comm_tier_bitmap = 0;  // bit0=WiFi6E, bit1=5G, bit2=LoRa
    
    // Methods
    bool hasCapability(const CapabilityBitmap& required) const;
    uint8_t getCommTierCount() const;
    std::string toString() const;
    std::vector<std::string> getCapabilitiesList() const;
    
    // Serialization
    std::string toJson() const;
    static std::optional<CapabilityBitmap> fromJson(const std::string& json);
    std::string toYaml() const;
    static std::optional<CapabilityBitmap> fromYaml(const std::string& yaml);
};

// Bid scoring for CBBA
class BidScorer {
public:
    struct Weights {
        float distance_weight = 1.0f;      // Distance to task
        float capability_weight = 2.0f;    // Capability match
        float energy_weight = 1.5f;        // Energy efficiency
        float time_weight = 1.0f;          // Time to complete
        float risk_weight = 1.5f;          // Risk assessment
    };
    
    struct BidScore {
        float total_cost = 0.0f;
        float distance_cost = 0.0f;
        float capability_cost = 0.0f;
        float energy_cost = 0.0f;
        float time_cost = 0.0f;
        float risk_cost = 0.0f;
        bool feasible = true;
        std::string infeasible_reason;
    };
    
    // Compute bid cost for a drone-task pair
    static BidScore computeBid(const CapabilityBitmap& drone_caps,
                               const AgentState& drone_state,
                               const Task& task,
                               const Weights& weights = Weights{},
                               const SwarmMap* map = nullptr);
    
    // Distance cost (normalized)
    static float computeDistanceCost(const Vector3& drone_pos, 
                                      const TaskGeometry& geometry);
    
    // Capability match cost (0 = perfect match, higher = less capable)
    static float computeCapabilityCost(const CapabilityBitmap& drone_caps,
                                        const Task& task);
    
    // Energy cost (normalized, considers remaining battery)
    static float computeEnergyCost(const AgentState& state, 
                                    const Task& task,
                                    const SwarmMap* map = nullptr);
    
    // Time cost (normalized)
    static float computeTimeCost(const AgentState& state,
                                  const Task& task,
                                  const SwarmMap* map = nullptr);
    
    // Risk cost (obstacles, no-fly zones, weather)
    static float computeRiskCost(const AgentState& state,
                                  const Task& task,
                                  const SwarmMap* map = nullptr);
    
    // Heterogeneous bidding adjustments
    static float applyHeterogeneousAdjustment(const CapabilityBitmap& drone_caps,
                                               const Task& task,
                                               float base_cost);
    
private:
    static float normalize(float value, float min_val, float max_val);
    static float estimateTaskDuration(const Task& task, const CapabilityBitmap& caps);
    static float estimateEnergyConsumption(const Task& task, const CapabilityBitmap& caps);
};

} // namespace swarm::autonomy

#endif // CAPABILITY_HPP