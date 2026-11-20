#include "resync.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <chrono>
#include <random>
#include <sstream>
#include <iomanip>

namespace swarm::comm {

ConsensusResync::ConsensusResync(ConsensusEngine& consensus, MeshManager& mesh, const Config& config)
    : _consensus(consensus), _mesh(mesh), _config(config) {
}

ConsensusResync::~ConsensusResync() {
    // Stop any running resync
}

bool ConsensusResync::triggerResync(uint16_t peer_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    if (_active_resyncs.find(peer_id) != _active_resyncs.end()) {
        return false;  // Already resyncing
    }
    
    ResyncState state;
    state.phase = ResyncState::Phase::REQUESTING_STATE;
    state.resync_id = _next_resync_id++;
    state.peer_id = peer_id;
    state.start_time_us = Timestamp::nowUs();
    state.deadline_us = state.start_time_us + static_cast<uint64_t>(_config.max_resync_time_s * 1e6);
    state.retries = 0;
    
    _active_resyncs[peer_id] = state;
    
    // Request state from peer
    requestState(peer_id, state.resync_id);
    
    return true;
}

void ConsensusResync::triggerFullResync() {
    std::lock_guard<std::mutex> lock(_mutex);
    
    // Get all neighbors
    auto neighbors = _mesh.getNeighbors();
    
    for (const auto& neighbor : neighbors) {
        if (neighbor.is_direct) {
            triggerResync(neighbor.sysid);
        }
    }
}

void ConsensusResync::handleStateData(uint16_t src_id, const std::vector<uint8_t>& state_data) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    auto it = _active_resyncs.find(src_id);
    if (it == _active_resyncs.end()) {
        // Unsolicited state - might be push update
        mergeState(src_id, state_data);
        return;
    }
    
    ResyncState& state = it->second;
    
    if (state.phase != ResyncState::Phase::RECEIVING_STATE) {
        return;
    }
    
    // Store received state
    state.received_state_hash = computeStateHashFromData(state_data);
    
    // Merge state
    if (mergeState(src_id, state_data)) {
        state.phase = ResyncState::Phase::MERGING;
        
        // Verify merged state
        if (verifyState(src_id)) {
            state.phase = ResyncState::Phase::VERIFYING;
            state.phase = ResyncState::Phase::COMPLETE;
            
            // Update stats
            {
                std::lock_guard<std::mutex> lock(_stats_mutex);
                _stats.successful_resyncs++;
                _stats.total_resyncs++;
                float elapsed = (Timestamp::nowUs() - state.start_time_us) / 1e6f;
                _stats.avg_resync_time_s = 0.9f * _stats.avg_resync_time_s + 0.1f * elapsed;
            }
            
            if (_callback) {
                _callback(src_id, true, "Resync completed successfully");
            }
        } else {
            state.phase = ResyncState::Phase::FAILED;
            state.error_message = "State verification failed";
            
            if (state.retries < _config.max_state_request_retries) {
                // Retry
                state.retries++;
                state.phase = ResyncState::Phase::REQUESTING_STATE;
                requestState(src_id, state.resync_id);
            } else {
                {
                    std::lock_guard<std::mutex> lock(_stats_mutex);
                    _stats.failed_resyncs++;
                    _stats.total_resyncs++;
                }
                
                if (_callback) {
                    _callback(src_id, false, "State verification failed after retries");
                }
            }
        }
    }
}

void ConsensusResync::handleStateRequest(uint16_t requester_id, uint32_t resync_id) {
    // Send our state to requester
    sendState(requester_id, resync_id);
}

bool ConsensusResync::isResyncing() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return !_active_resyncs.empty();
}

std::optional<ConsensusResync::ResyncState> ConsensusResync::getResyncState(uint16_t peer_id) const {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _active_resyncs.find(peer_id);
    if (it != _active_resyncs.end()) {
        return it->second;
    }
    return std::nullopt;
}

const ConsensusResync::ResyncStats& ConsensusResync::getStats() const {
    return _stats;
}

void ConsensusResync::setResyncCallback(ResyncCallback cb) {
    _callback = cb;
}

void ConsensusResync::requestState(uint16_t peer_id, uint32_t resync_id) {
    // Send state request via mesh
    // Would serialize resync_id and send via mesh
    // For now, just log
}

void ConsensusResync::sendState(uint16_t peer_id, uint32_t resync_id) {
    // Serialize our state and send
    auto state_data = serializeState();
    // Send via mesh
    _mesh.sendMessage(peer_id, 1, SwarmMsgId::TASK_ALLOCATION, state_data.data(), state_data.size());
}

bool ConsensusResync::mergeState(uint16_t peer_id, const std::vector<uint8_t>& state_data) {
    // Deserialize and merge state with our consensus engine
    // For now, just verify hash
    std::vector<uint8_t> our_hash = computeStateHash();
    std::vector<uint8_t> their_hash = computeStateHashFromData(state_data);
    
    if (our_hash == their_hash) {
        return true;  // Already in sync
    }
    
    // In real implementation, would merge CBBA bundles, task states, etc.
    // For now, assume merge succeeds
    return true;
}

bool ConsensusResync::verifyState(uint16_t peer_id) {
    // Verify merged state is consistent
    // Would check CBBA convergence, task assignments, etc.
    return true;
}

std::vector<uint8_t> ConsensusResync::computeStateHash() const {
    // Compute hash of current consensus state
    // Would hash CBBA bundles, task assignments, etc.
    std::vector<uint8_t> hash(32);
    // Simplified: just return some hash
    return hash;
}

std::vector<uint8_t> ConsensusResync::computeStateHashFromData(const std::vector<uint8_t>& data) const {
    // Compute SHA256 or similar
    std::vector<uint8_t> hash(32);
    return hash;
}

void ConsensusResync::handleTimeout(uint16_t peer_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    auto it = _active_resyncs.find(peer_id);
    if (it == _active_resyncs.end()) return;
    
    ResyncState& state = it->second;
    state.phase = ResyncState::Phase::FAILED;
    state.error_message = "Resync timeout";
    
    if (state.retries < _config.max_state_request_retries) {
        state.retries++;
        state.phase = ResyncState::Phase::REQUESTING_STATE;
        state.start_time_us = Timestamp::nowUs();
        state.deadline_us = state.start_time_us + static_cast<uint64_t>(_config.max_resync_time_s * 1e6);
        requestState(peer_id, state.resync_id);
    } else {
        {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.failed_resyncs++;
            _stats.total_resyncs++;
        }
        
        if (_callback) {
            _callback(peer_id, false, "Resync timeout after max retries");
        }
        
        _active_resyncs.erase(it);
    }
}

std::vector<uint8_t> ConsensusResync::serializeState() const {
    // Serialize CBBA state, task assignments, etc.
    std::vector<uint8_t> data;
    // Would serialize all relevant state
    return data;
}

bool ConsensusResync::deserializeState(const std::vector<uint8_t>& data) {
    // Deserialize and merge
    return true;
}

uint64_t ConsensusResync::nowUs() const {
    return Timestamp::nowUs();
}

float ConsensusResync::elapsedMs(uint64_t start_us) const {
    return (Timestamp::nowUs() - start_us) / 1000.0f;
}

} // namespace swarm::comm