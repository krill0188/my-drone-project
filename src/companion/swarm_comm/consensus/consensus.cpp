#include "consensus.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <random>
#include <cstring>

namespace swarm::comm {

ConsensusEngine::ConsensusEngine(MeshManager& mesh, uint16_t own_id)
    : _mesh(mesh), _own_id(own_id) {
    
    // Register callbacks for different message types
    _mesh.registerCallback(SwarmMsgId::TASK_BID,
        [this](uint16_t src_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len) {
            handleCbbaMessage(0, src_id, std::vector<uint8_t>(data, data + len));
        });
    
    _mesh.registerCallback(SwarmMsgId::TASK_AWARD,
        [this](uint16_t src_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len) {
            // Handle task award
        });
    
    // Register for custom consensus messages (would need new MAVLink message IDs)
    // For now, using existing message types
}

uint32_t ConsensusEngine::startConsensus(Type type, const std::vector<uint16_t>& participants,
                                        const std::vector<uint8_t>& proposal_data,
                                        float timeout_s) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    uint32_t consensus_id = _next_consensus_id++;
    
    ConsensusState state;
    state.consensus_id = consensus_id;
    state.initiator_id = _own_id;
    state.type = type;
    state.status = ConsensusState::PENDING;
    state.participants = participants;
    state.votes_received.clear();
    state.consensus_data = proposal_data;
    state.start_time_us = Timestamp::nowUs();
    state.deadline_us = state.start_time_us + static_cast<uint64_t>(timeout_s * 1e6);
    
    // Required votes: 2/3 majority + 1
    state.required_votes = (participants.size() * 2 / 3) + 1;
    
    _active_consensus[consensus_id] = state;
    
    // Broadcast proposal to all participants
    broadcastProposal(consensus_id);
    
    return consensus_id;
}

void ConsensusEngine::handleVote(uint32_t consensus_id, uint16_t voter_id, bool vote,
                                const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    auto it = _active_consensus.find(consensus_id);
    if (it == _active_consensus.end()) return;
    
    ConsensusState& state = it->second;
    
    // Check if voter is a participant
    if (std::find(state.participants.begin(), state.participants.end(), voter_id) == state.participants.end()) {
        return;
    }
    
    // Check if already voted
    if (std::find(state.votes_received.begin(), state.votes_received.end(), voter_id) != state.votes_received.end()) {
        return;
    }
    
    if (vote) {
        state.votes_received.push_back(voter_id);
    }
    
    // Check if consensus reached
    if (state.votes_received.size() >= state.required_votes) {
        finalizeConsensus(consensus_id, true);
    }
}

void ConsensusEngine::handleProposal(uint32_t consensus_id, uint16_t proposer_id,
                                    const std::vector<uint8_t>& proposal_data) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    auto it = _active_consensus.find(consensus_id);
    if (it == _active_consensus.end()) {
        // New consensus proposal from another drone
        // Check if we're a participant (for now, accept if we're in swarm)
        // In real implementation, would check participant list
        
        // Auto-vote yes for emergency, evaluate for others
        bool vote = true;  // Simplified
        handleVote(consensus_id, _own_id, vote);
    }
}

std::optional<ConsensusEngine::ConsensusState> ConsensusEngine::getConsensusState(uint32_t consensus_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _active_consensus.find(consensus_id);
    if (it != _active_consensus.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool ConsensusEngine::cancelConsensus(uint32_t consensus_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _active_consensus.find(consensus_id);
    if (it != _active_consensus.end()) {
        if (it->second.initiator_id == _own_id) {
            it->second.status = ConsensusState::FAILED;
            finalizeConsensus(consensus_id, false);
            return true;
        }
    }
    return false;
}

void ConsensusEngine::update() {
    std::lock_guard<std::mutex> lock(_mutex);
    cleanupExpiredConsensus();
    
    // Process votes for pending consensus
    for (auto& [id, state] : _active_consensus) {
        if (state.status == ConsensusState::PENDING || state.status == ConsensusState::VOTING) {
            processVotes(id);
        }
    }
}

void ConsensusEngine::cleanupExpiredConsensus() {
    uint64_t now = Timestamp::nowUs();
    
    for (auto it = _active_consensus.begin(); it != _active_consensus.end();) {
        if (now > it->second.deadline_us) {
            if (it->second.status == ConsensusState::PENDING || it->second.status == ConsensusState::VOTING) {
                finalizeConsensus(it->first, false);
            }
            // Keep for a bit for logging, then erase
            if (now > it->second.deadline_us + 5000000) {  // 5 seconds after deadline
                it = _active_consensus.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }
}

void ConsensusEngine::broadcastProposal(uint32_t consensus_id) {
    auto it = _active_consensus.find(consensus_id);
    if (it == _active_consensus.end()) return;
    
    ConsensusState& state = it->second;
    state.status = ConsensusState::VOTING;
    
    // Serialize proposal
    std::vector<uint8_t> msg;
    msg.push_back(static_cast<uint8_t>(state.type));
    // Would serialize consensus_id, proposal_data, etc.
    
    // Send to all participants
    for (uint16_t participant : state.participants) {
        if (participant != _own_id) {
            _mesh.sendMessage(participant, SwarmMsgId::TASK_BID, msg.data(), msg.size());
        }
    }
    
    // Self-vote
    handleVote(state.consensus_id, _own_id, true);
}

void ConsensusEngine::processVotes(uint32_t consensus_id) {
    auto it = _active_consensus.find(consensus_id);
    if (it == _active_consensus.end()) return;
    
    ConsensusState& state = it->second;
    
    if (state.votes_received.size() >= state.required_votes) {
        finalizeConsensus(consensus_id, true);
    }
}

bool ConsensusEngine::checkConsensusReached(uint32_t consensus_id) {
    auto it = _active_consensus.find(consensus_id);
    if (it == _active_consensus.end()) return false;
    return it->second.votes_received.size() >= it->second.required_votes;
}

void ConsensusEngine::finalizeConsensus(uint32_t consensus_id, bool success) {
    auto it = _active_consensus.find(consensus_id);
    if (it == _active_consensus.end()) return;
    
    ConsensusState& state = it->second;
    state.status = success ? ConsensusState::REACHED : ConsensusState::FAILED;
    
    // Call callback
    if (_callback) {
        _callback(consensus_id, success, state.consensus_data);
    }
    
    // For task allocation, would publish awards
    // For trajectory, would execute replan
    // For formation, would execute geometry change
}

void ConsensusEngine::sendVote(uint32_t consensus_id, uint16_t target_id, bool vote) {
    // Send vote message
    std::vector<uint8_t> msg;
    msg.push_back(vote ? 1 : 0);
    _mesh.sendMessage(target_id, SwarmMsgId::TASK_AWARD, msg.data(), msg.size());
}

// CBBA Task Allocation
uint32_t ConsensusEngine::startTaskAllocationConsensus(const std::vector<std::array<uint8_t, 16>>& tasks,
                                                      const std::vector<uint16_t>& participants) {
    std::vector<uint8_t> data;
    // Serialize tasks
    for (const auto& task : tasks) {
        data.insert(data.end(), task.begin(), task.end());
    }
    return startConsensus(Type::TASK_ALLOCATION, participants, data, 10.0f);
}

void ConsensusEngine::handleCbbaMessage(uint32_t consensus_id, uint16_t sender_id,
                                       const std::vector<uint8_t>& data) {
    // Handle CBBA bid/award messages
    // Parse bid data and process
    handleVote(consensus_id, sender_id, true, data);
}

// Trajectory Replan Consensus
uint32_t ConsensusEngine::startTrajectoryConsensus(const planner::Trajectory& trajectory,
                                                  const std::vector<uint16_t>& participants) {
    std::vector<uint8_t> data = serializeTrajectory(trajectory);
    return startConsensus(Type::TRAJECTORY_REPLAN, participants, data, 2.0f);  // Fast timeout for replan
}

void ConsensusEngine::handleTrajectoryMessage(uint32_t consensus_id, uint16_t sender_id,
                                             const std::vector<uint8_t>& data) {
    auto traj = deserializeTrajectory(data);
    if (traj) {
        // Process trajectory proposal
        handleVote(consensus_id, sender_id, true, data);
    }
}

// Formation Consensus
uint32_t ConsensusEngine::startFormationConsensus(const formation::FormationGeometry& geometry,
                                                 const std::vector<uint16_t>& participants) {
    std::vector<uint8_t> data = serializeFormation(geometry);
    return startConsensus(Type::FORMATION_CHANGE, participants, data, 5.0f);
}

void ConsensusEngine::handleFormationMessage(uint32_t consensus_id, uint16_t sender_id,
                                            const std::vector<uint8_t>& data) {
    auto geom = deserializeFormation(data);
    if (geom) {
        handleVote(consensus_id, sender_id, true, data);
    }
}

// Emergency Consensus
uint32_t ConsensusEngine::startEmergencyConsensus(uint8_t emergency_type,
                                                 const std::vector<uint16_t>& participants) {
    std::vector<uint8_t> data = {emergency_type};
    return startConsensus(Type::EMERGENCY, participants, data, 1.0f);  // Very fast timeout
}

void ConsensusEngine::handleEmergencyMessage(uint32_t consensus_id, uint16_t sender_id,
                                            const std::vector<uint8_t>& data) {
    // Emergency consensus: auto-vote yes
    handleVote(consensus_id, _own_id, true);
}

// Serialization
std::vector<uint8_t> ConsensusEngine::serializeTrajectory(const planner::Trajectory& traj) {
    std::vector<uint8_t> data;
    
    // Number of points
    uint32_t num_points = traj.points.size();
    data.insert(data.end(), reinterpret_cast<uint8_t*>(&num_points), 
                reinterpret_cast<uint8_t*>(&num_points) + 4);
    
    for (const auto& pt : traj.points) {
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(&pt.position), 
                    reinterpret_cast<const uint8_t*>(&pt.position) + sizeof(planner::Vector3));
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(&pt.velocity),
                    reinterpret_cast<const uint8_t*>(&pt.velocity) + sizeof(planner::Vector3));
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(&pt.acceleration),
                    reinterpret_cast<const uint8_t*>(&pt.acceleration) + sizeof(planner::Vector3));
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(&pt.yaw),
                    reinterpret_cast<const uint8_t*>(&pt.yaw) + 4);
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(&pt.timestamp_us),
                    reinterpret_cast<const uint8_t*>(&pt.timestamp_us) + 8);
    }
    
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&traj.total_time),
                reinterpret_cast<const uint8_t*>(&traj.total_time) + 4);
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&traj.total_distance),
                reinterpret_cast<const uint8_t*>(&traj.total_distance) + 4);
    
    return data;
}

std::optional<planner::Trajectory> ConsensusEngine::deserializeTrajectory(const std::vector<uint8_t>& data) {
    if (data.size() < 4) return std::nullopt;
    
    planner::Trajectory traj;
    size_t offset = 0;
    
    uint32_t num_points;
    std::memcpy(&num_points, data.data() + offset, 4);
    offset += 4;
    
    traj.points.resize(num_points);
    
    for (uint32_t i = 0; i < num_points; ++i) {
        if (offset + sizeof(planner::Vector3) * 3 + 4 + 8 > data.size()) return std::nullopt;
        
        std::memcpy(&traj.points[i].position, data.data() + offset, sizeof(planner::Vector3));
        offset += sizeof(planner::Vector3);
        std::memcpy(&traj.points[i].velocity, data.data() + offset, sizeof(planner::Vector3));
        offset += sizeof(planner::Vector3);
        std::memcpy(&traj.points[i].acceleration, data.data() + offset, sizeof(planner::Vector3));
        offset += sizeof(planner::Vector3);
        std::memcpy(&traj.points[i].yaw, data.data() + offset, 4);
        offset += 4;
        std::memcpy(&traj.points[i].timestamp_us, data.data() + offset, 8);
        offset += 8;
    }
    
    if (offset + 8 > data.size()) return std::nullopt;
    std::memcpy(&traj.total_time, data.data() + offset, 4);
    offset += 4;
    std::memcpy(&traj.total_distance, data.data() + offset, 4);
    
    return traj;
}

std::vector<uint8_t> ConsensusEngine::serializeFormation(const formation::FormationGeometry& geom) {
    std::vector<uint8_t> data;
    
    data.push_back(static_cast<uint8_t>(geom.type));
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&geom.spacing_m), 
                reinterpret_cast<const uint8_t*>(&geom.spacing_m) + 4);
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&geom.altitude_offset_m),
                reinterpret_cast<const uint8_t*>(&geom.altitude_offset_m) + 4);
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&geom.heading_offset_deg),
                reinterpret_cast<const uint8_t*>(&geom.heading_offset_deg) + 4);
    
    uint32_t num_positions = geom.relative_positions.size();
    data.insert(data.end(), reinterpret_cast<const uint8_t*>(&num_positions),
                reinterpret_cast<const uint8_t*>(&num_positions) + 4);
    
    for (const auto& pos : geom.relative_positions) {
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(&pos),
                    reinterpret_cast<const uint8_t*>(&pos) + sizeof(formation::Vector3));
    }
    
    return data;
}

std::optional<formation::FormationGeometry> ConsensusEngine::deserializeFormation(const std::vector<uint8_t>& data) {
    if (data.size() < 13) return std::nullopt;  // Minimum size
    
    formation::FormationGeometry geom;
    size_t offset = 0;
    
    geom.type = static_cast<formation::GeometryType>(data[offset++]);
    
    if (offset + 4 + 4 + 4 > data.size()) return std::nullopt;
    std::memcpy(&geom.spacing_m, data.data() + offset, 4); offset += 4;
    std::memcpy(&geom.altitude_offset_m, data.data() + offset, 4); offset += 4;
    std::memcpy(&geom.heading_offset_deg, data.data() + offset, 4); offset += 4;
    
    if (offset + 4 > data.size()) return std::nullopt;
    uint32_t num_positions;
    std::memcpy(&num_positions, data.data() + offset, 4); offset += 4;
    
    geom.relative_positions.resize(num_positions);
    for (uint32_t i = 0; i < num_positions; ++i) {
        if (offset + sizeof(formation::Vector3) > data.size()) return std::nullopt;
        std::memcpy(&geom.relative_positions[i], data.data() + offset, sizeof(formation::Vector3));
        offset += sizeof(formation::Vector3);
    }
    
    return geom;
}

} // namespace swarm::comm