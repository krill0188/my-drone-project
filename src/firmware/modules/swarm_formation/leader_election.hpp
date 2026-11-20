#ifndef LEADER_ELECTION_HPP
#define LEADER_ELECTION_HPP

#include <cstdint>
#include <vector>
#include <optional>
#include "swarm_comm.hpp"

namespace swarm::formation {

// Bully Algorithm for leader election
class LeaderElection {
public:
    LeaderElection(comm::MeshManager& mesh, uint16_t own_id, uint8_t priority = 100)
        : _mesh(mesh), _own_id(own_id), _priority(priority) {}
    
    // Initialize election
    void init();
    
    // Start election process
    void startElection();
    
    // Handle incoming election messages
    void handleElectionMessage(uint16_t sender_id, uint8_t sender_priority, bool is_election, bool is_victory);
    
    // Check if we are the leader
    bool isLeader() const { return _is_leader; }
    
    // Get current leader ID
    uint16_t getLeaderId() const { return _leader_id; }
    
    // Get election state
    enum class ElectionState {
        IDLE,
        ELECTION_IN_PROGRESS,
        LEADER_ELECTED,
        WAITING_FOR_VICTORY
    };
    
    ElectionState getState() const { return _state; }
    
    // Called periodically
    void update();

private:
    comm::MeshManager& _mesh;
    uint16_t _own_id;
    uint8_t _priority;
    
    ElectionState _state = ElectionState::IDLE;
    bool _is_leader = false;
    uint16_t _leader_id = 0;
    uint64_t _election_start_us = 0;
    uint64_t _last_leader_heartbeat_us = 0;
    uint32_t _election_timeout_ms = 2000;
    
    void sendElectionMessage();
    void sendVictoryMessage();
    void becomeLeader();
    void becomeFollower(uint16_t new_leader_id);
    bool isHigherPriority(uint16_t other_id, uint8_t other_priority) const;
};

} // namespace swarm::formation

#endif // LEADER_ELECTION_HPP