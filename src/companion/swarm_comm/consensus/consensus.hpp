#ifndef CONSENSUS_HPP
#define CONSENSUS_HPP

#include <cstdint>
#include <vector>
#include <array>
#include <functional>
#include <unordered_map>
#include <mutex>
#include "swarm_comm.hpp"
#include "trajectory_planner.hpp"

namespace swarm::comm {

// Consensus protocol for decentralized decision making
// Supports CBBA for task allocation and trajectory consensus for replanning

class ConsensusEngine {
public:
    using ConsensusCallback = std::function<void(uint32_t consensus_id, bool reached, const std::vector<uint8_t>& data)>;
    
    struct ConsensusState {
        uint32_t consensus_id;
        uint16_t initiator_id;
        enum Type { TASK_ALLOCATION, TRAJECTORY_REPLAN, FORMATION_CHANGE, EMERGENCY } type;
        enum Status { PENDING, VOTING, REACHED, FAILED, TIMEOUT } status;
        std::vector<uint16_t> participants;
        std::vector<uint16_t> votes_received;
        std::vector<uint8_t> consensus_data;
        uint64_t start_time_us;
        uint64_t deadline_us;
        uint8_t required_votes;  // e.g., 2/3 majority
    };
    
    ConsensusEngine(MeshManager& mesh, uint16_t own_id);
    
    // Start a new consensus round
    uint32_t startConsensus(Type type, const std::vector<uint16_t>& participants,
                           const std::vector<uint8_t>& proposal_data,
                           float timeout_s = 5.0f);
    
    // Handle incoming vote
    void handleVote(uint32_t consensus_id, uint16_t voter_id, bool vote, 
                    const std::vector<uint8_t>& data = {});
    
    // Handle incoming proposal
    void handleProposal(uint32_t consensus_id, uint16_t proposer_id,
                       const std::vector<uint8_t>& proposal_data);
    
    // Check consensus status
    std::optional<ConsensusState> getConsensusState(uint32_t consensus_id);
    
    // Cancel consensus
    bool cancelConsensus(uint32_t consensus_id);
    
    // Periodic update (call at 10 Hz)
    void update();
    
    // Register callback for consensus results
    void registerCallback(ConsensusCallback cb) { _callback = cb; }

private:
    MeshManager& _mesh;
    uint16_t _own_id;
    std::mutex _mutex;
    std::unordered_map<uint32_t, ConsensusState> _active_consensus;
    uint32_t _next_consensus_id = 1;
    ConsensusCallback _callback;
    
    // CBBA-specific for task allocation
    struct CbbaState {
        std::vector<std::array<uint8_t, 16>> bundle;
        std::vector<float> costs;
        std::unordered_map<uint16_t, std::vector<std::array<uint8_t, 16>>> neighbor_bundles;
        uint32_t iteration = 0;
    };
    
    // Task allocation consensus (CBBA)
    uint32_t startTaskAllocationConsensus(const std::vector<std::array<uint8_t, 16>>& tasks,
                                         const std::vector<uint16_t>& participants);
    void handleCbbaMessage(uint32_t consensus_id, uint16_t sender_id, 
                          const std::vector<uint8_t>& data);
    
    // Trajectory replan consensus
    uint32_t startTrajectoryConsensus(const planner::Trajectory& trajectory,
                                     const std::vector<uint16_t>& participants);
    void handleTrajectoryMessage(uint32_t consensus_id, uint16_t sender_id,
                                const std::vector<uint8_t>& data);
    
    // Formation change consensus
    uint32_t startFormationConsensus(const formation::FormationGeometry& geometry,
                                    const std::vector<uint16_t>& participants);
    void handleFormationMessage(uint32_t consensus_id, uint16_t sender_id,
                               const std::vector<uint8_t>& data);
    
    // Emergency consensus (highest priority)
    uint32_t startEmergencyConsensus(uint8_t emergency_type,
                                    const std::vector<uint16_t>& participants);
    void handleEmergencyMessage(uint32_t consensus_id, uint16_t sender_id,
                               const std::vector<uint8_t>& data);
    
    // Helper functions
    void broadcastProposal(uint32_t consensus_id);
    void processVotes(uint32_t consensus_id);
    bool checkConsensusReached(uint32_t consensus_id);
    void finalizeConsensus(uint32_t consensus_id, bool success);
    void sendVote(uint32_t consensus_id, uint16_t target_id, bool vote);
    void cleanupExpiredConsensus();
    
    // Serialization
    std::vector<uint8_t> serializeTrajectory(const planner::Trajectory& traj);
    std::optional<planner::Trajectory> deserializeTrajectory(const std::vector<uint8_t>& data);
    std::vector<uint8_t> serializeFormation(const formation::FormationGeometry& geom);
    std::optional<formation::FormationGeometry> deserializeFormation(const std::vector<uint8_t>& data);
};

} // namespace swarm::comm

#endif // CONSENSUS_HPP