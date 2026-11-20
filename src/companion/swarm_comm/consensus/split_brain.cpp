#include "split_brain.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <chrono>
#include <queue>
#include <iostream>

namespace swarm::comm {

SplitBrainDetector::SplitBrainDetector(ConsensusEngine& consensus, MeshManager& mesh, const Config& config)
    : _consensus(consensus), _mesh(mesh), _config(config) {
}

SplitBrainDetector::~SplitBrainDetector() {
    stop();
}

void SplitBrainDetector::start() {
    if (_running.load()) return;
    
    _running.store(true);
    _detector_thread = std::thread(&SplitBrainDetector::detectorLoop, this);
}

void SplitBrainDetector::stop() {
    if (!_running.load()) return;
    
    _running.store(false);
    
    if (_detector_thread.joinable()) {
        _detector_thread.join();
    }
}

void SplitBrainDetector::checkForPartitions() {
    // Manual trigger for partition check
    auto partitions = detectPartitions();
    
    std::lock_guard<std::mutex> lock(_mutex);
    for (const auto& partition : partitions) {
        PartitionInfo info;
        info.nodes = partition;
        info.detected_time_us = Timestamp::nowUs();
        info.node_count = partition.size();
        
        // Compute state hash
        info.state_hash = computePartitionHash(partition);
        
        // Check quorum
        info.has_quorum = hasQuorum(info);
        
        // Check if new partition
        uint16_t rep = partition[0];  // Use first node as representative
        auto it = _partitions.find(rep);
        if (it == _partitions.end()) {
            // New partition detected
            _partitions[rep] = info;
            
            if (_partition_cb) {
                _partition_cb(info);
            }
            
            {
                std::lock_guard<std::mutex> lock(_stats_mutex);
                _stats.partitions_detected++;
            }
        }
    }
}

void SplitBrainDetector::handleMerge(const std::vector<uint16_t>& nodes_a, const std::vector<uint16_t>& nodes_b) {
    // Find partitions containing these nodes
    uint16_t rep_a = 0, rep_b = 0;
    
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (const auto& [rep, info] : _partitions) {
            if (std::find(info.nodes.begin(), info.nodes.end(), nodes_a[0]) != info.nodes.end()) {
                rep_a = rep;
            }
            if (std::find(info.nodes.begin(), info.nodes.end(), nodes_b[0]) != info.nodes.end()) {
                rep_b = rep;
            }
        }
    }
    
    if (rep_a == 0 || rep_b == 0 || rep_a == rep_b) return;
    
    // Check if can merge
    std::lock_guard<std::mutex> lock(_mutex);
    auto it_a = _partitions.find(rep_a);
    auto it_b = _partitions.find(rep_b);
    
    if (it_a == _partitions.end() || it_b == _partitions.end()) return;
    
    if (!canMerge(it_a->second, it_b->second)) {
        return;
    }
    
    // Merge partitions
    if (mergePartitions(it_a->second, it_b->second)) {
        // Record merge event
        MergeEvent merge;
        merge.timestamp_us = Timestamp::nowUs();
        merge.partition_a_nodes = it_a->second.nodes;
        merge.partition_b_nodes = it_b->second.nodes;
        merge.reconciled = true;
        merge.reconciliation_method = "automatic";
        
        _merge_history.push_back(merge);
        
        {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.merges_completed++;
        }
        
        if (_merge_cb) {
            _merge_cb(merge);
        }
    }
}

bool SplitBrainDetector::forceReconciliation(const std::vector<uint16_t>& nodes) {
    // Force state reconciliation for given nodes
    // Would run consensus reconciliation
    return true;
}

std::vector<SplitBrainDetector::PartitionInfo> SplitBrainDetector::getPartitions() const {
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<PartitionInfo> result;
    result.reserve(_partitions.size());
    for (const auto& [rep, info] : _partitions) {
        result.push_back(info);
    }
    return result;
}

std::vector<SplitBrainDetector::MergeEvent> SplitBrainDetector::getMergeHistory() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _merge_history;
}

const SplitBrainDetector::Stats& SplitBrainDetector::getStats() const {
    return _stats;
}

void SplitBrainDetector::setPartitionCallback(PartitionCallback cb) {
    _partition_cb = cb;
}

void SplitBrainDetector::setMergeCallback(MergeCallback cb) {
    _merge_cb = cb;
}

void SplitBrainDetector::detectorLoop() {
    using namespace std::chrono;
    
    while (_running.load()) {
        auto start = steady_clock::now();
        
        checkForPartitions();
        
        // Check for merges (partitions that are now connected)
        // Would check connectivity between partitions
        
        std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(_config.partition_detection_timeout_s)));
    }
}

std::vector<std::vector<uint16_t>> SplitBrainDetector::detectPartitions() {
    // Get all connected nodes from mesh
    auto neighbors = _mesh.getNeighbors();
    std::vector<uint16_t> all_nodes;
    all_nodes.push_back(_mesh.getOwnId());
    for (const auto& n : neighbors) {
        all_nodes.push_back(n.sysid);
    }
    
    // Build adjacency graph
    std::unordered_map<uint16_t, std::vector<uint16_t>> adj;
    for (uint16_t node : all_nodes) {
        adj[node] = {};
    }
    
    // Get connectivity from mesh
    auto neighbors = _mesh.getNeighbors();
    for (const auto& n : neighbors) {
        adj[n.sysid].push_back(n.sysid);
        // In real implementation, would get full topology
    }
    
    // Find connected components (partitions)
    std::vector<std::vector<uint16_t>> partitions;
    std::unordered_map<uint16_t, bool> visited;
    
    for (uint16_t node : all_nodes) {
        if (visited[node]) continue;
        
        std::vector<uint16_t> partition;
        std::queue<uint16_t> q;
        q.push(node);
        visited[node] = true;
        
        while (!q.empty()) {
            uint16_t current = q.front(); q.pop();
            partition.push_back(current);
            
            for (uint16_t neighbor : adj[current]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }
        
        if (!partition.empty()) {
            partitions.push_back(partition);
        }
    }
    
    return partitions;
}

bool SplitBrainDetector::canMerge(const PartitionInfo& a, const PartitionInfo& b) {
    // Check if both have quorum
    if (!a.has_quorum || !b.has_quorum) return false;
    
    // Check state divergence
    if (a.state_hash == b.state_hash) return true;
    
    // Check divergence threshold
    // Would compare state hashes
    return true;  // Simplified
}

bool SplitBrainDetector::mergePartitions(const PartitionInfo& a, const PartitionInfo& b) {
    // Remove both partitions
    uint16_t rep_a = a.nodes.empty() ? 0 : a.nodes[0];
    uint16_t rep_b = b.nodes.empty() ? 0 : b.nodes[0];
    
    if (rep_a == 0 || rep_b == 0) return false;
    
    // Merge node lists
    std::vector<uint16_t> merged_nodes = a.nodes;
    merged_nodes.insert(merged_nodes.end(), b.nodes.begin(), b.nodes.end());
    
    // Remove duplicates
    std::sort(merged_nodes.begin(), merged_nodes.end());
    merged_nodes.erase(std::unique(merged_nodes.begin(), merged_nodes.end()), merged_nodes.end());
    
    // Create merged partition
    PartitionInfo merged;
    merged.nodes = merged_nodes;
    merged.detected_time_us = Timestamp::nowUs();
    merged.node_count = merged_nodes.size();
    merged.state_hash = computePartitionHash(merged_nodes);
    merged.has_quorum = hasQuorum(merged);
    
    // Replace partitions
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _partitions.erase(a.nodes[0]);
        _partitions.erase(b.nodes[0]);
        _partitions[merged.nodes[0]] = merged;
    }
    
    return true;
}

bool SplitBrainDetector::reconcileState(const PartitionInfo& a, const PartitionInfo& b) {
    // Trigger consensus reconciliation between partitions
    // Would invoke consensus engine to reconcile
    return true;
}

std::vector<uint8_t> SplitBrainDetector::computePartitionHash(const std::vector<uint16_t>& nodes) {
    // Compute hash of combined state of all nodes in partition
    std::vector<uint8_t> hash(32);
    // Simplified
    return hash;
}

bool SplitBrainDetector::hasQuorum(const PartitionInfo& partition) const {
    return partition.node_count >= _config.min_quorum_size;
}

uint64_t SplitBrainDetector::nowUs() const {
    return Timestamp::nowUs();
}

float SplitBrainDetector::elapsedS(uint64_t start_us) const {
    return (Timestamp::nowUs() - start_us) / 1e6f;
}

} // namespace swarm::comm