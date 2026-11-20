#include "swarm_formation.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <cmath>
#include <algorithm>
#include <deque>

namespace swarm::formation {

RelativeLocalizationFusion::RelativeLocalizationFusion()
    : _uwb_range_valid(false), _vision_valid(false), _lidar_valid(false),
      _last_uwb_update_us(0), _last_vision_update_us(0), _last_lidar_update_us(0) {}

void RelativeLocalizationFusion::updateUwbRange(uint16_t anchor_id, float range_m, float quality) {
    UwbMeasurement meas;
    meas.anchor_id = anchor_id;
    meas.range_m = range_m;
    meas.quality = quality;
    meas.timestamp_us = Timestamp::nowUs();
    
    _uwb_measurements[anchor_id] = meas;
    _last_uwb_update_us = meas.timestamp_us;
    _uwb_range_valid = true;
}

void RelativeLocalizationFusion::updateVisionRelativePose(uint16_t target_id, 
                                                           const Vector3& position_flu,
                                                           const Quaternion& orientation_flu,
                                                           float confidence) {
    VisionMeasurement meas;
    meas.target_id = target_id;
    meas.position_flu = position_flu;
    meas.orientation_flu = orientation_flu;
    meas.confidence = confidence;
    meas.timestamp_us = Timestamp::nowUs();
    
    _vision_measurements[target_id] = meas;
    _last_vision_update_us = meas.timestamp_us;
    _vision_valid = (confidence > 0.7f);
}

void RelativeLocalizationFusion::updateLidarScan(const std::vector<Vector3>& points_flu, 
                                                  float confidence) {
    LidarMeasurement meas;
    meas.points_flu = points_flu;
    meas.confidence = confidence;
    meas.timestamp_us = Timestamp::nowUs();
    
    _lidar_measurements.push_back(meas);
    if (_lidar_measurements.size() > 10) {
        _lidar_measurements.erase(_lidar_measurements.begin());
    }
    _last_lidar_update_us = meas.timestamp_us;
    _lidar_valid = (confidence > 0.7f);
}

Vector3 RelativeLocalizationFusion::getFusedPosition() const {
    Vector3 fused{0, 0, 0};
    float total_weight = 0.0f;
    
    // UWB weight
    if (_uwb_range_valid) {
        float uwb_weight = 0.0f;
        Vector3 uwb_pos{0, 0, 0};
        for (const auto& [id, meas] : _uwb_measurements) {
            uwb_weight += meas.quality;
            // Simplified: would trilaterate from multiple anchors
            // For now, just use first anchor
            if (uwb_weight == meas.quality) {
                uwb_pos = estimatePositionFromUwb(meas);
            }
        }
        if (uwb_weight > 0) {
            fused.x += uwb_pos.x * uwb_weight;
            fused.y += uwb_pos.y * uwb_weight;
            fused.z += uwb_pos.z * uwb_weight;
            total_weight += uwb_weight;
        }
    }
    
    // Vision weight
    if (_vision_valid) {
        for (const auto& [id, meas] : _vision_measurements) {
            float weight = meas.confidence * 2.0f;  // Vision typically more accurate
            fused.x += meas.position_flu.x * weight;
            fused.y += meas.position_flu.y * weight;
            fused.z += meas.position_flu.z * weight;
            total_weight += weight;
        }
    }
    
    // LiDAR weight
    if (_lidar_valid && !_lidar_measurements.empty()) {
        const auto& latest = _lidar_measurements.back();
        float weight = latest.confidence * 1.5f;
        // Simplified: use centroid of points
        Vector3 lidar_centroid = computeCentroid(latest.points_flu);
        fused.x += lidar_centroid.x * weight;
        fused.y += lidar_centroid.y * weight;
        fused.z += lidar_centroid.z * weight;
        total_weight += weight;
    }
    
    if (total_weight > 0.0f) {
        fused.x /= total_weight;
        fused.y /= total_weight;
        fused.z /= total_weight;
    }
    
    return fused;
}

bool RelativeLocalizationFusion::isValid() const {
    uint64_t now = Timestamp::nowUs();
    
    // Check if any sensor has recent data (< 1 second)
    bool uwb_recent = _uwb_range_valid && (now - _last_uwb_update_us < 1000000);
    bool vision_recent = _vision_valid && (now - _last_vision_update_us < 1000000);
    bool lidar_recent = _lidar_valid && (now - _last_lidar_update_us < 1000000);
    
    return uwb_recent || vision_recent || lidar_recent;
}

float RelativeLocalizationFusion::getPositionUncertainty() const {
    // Simplified uncertainty estimate based on sensor quality
    float uncertainty = 1.0f;  // meters
    
    if (_uwb_range_valid) {
        float min_quality = 1.0f;
        for (const auto& [id, meas] : _uwb_measurements) {
            min_quality = std::min(min_quality, meas.quality);
        }
        uncertainty = std::min(uncertainty, 0.5f / min_quality);
    }
    
    if (_vision_valid) {
        for (const auto& [id, meas] : _vision_measurements) {
            uncertainty = std::min(uncertainty, 0.3f / meas.confidence);
        }
    }
    
    if (_lidar_valid && !_lidar_measurements.empty()) {
        const auto& latest = _lidar_measurements.back();
        uncertainty = std::min(uncertainty, 0.2f / latest.confidence);
    }
    
    return uncertainty;
}

Vector3 RelativeLocalizationFusion::estimatePositionFromUwb(const UwbMeasurement& meas) const {
    // Simplified: assume anchor at origin, range gives distance
    // In real implementation, would use anchor positions and trilateration
    return Vector3{meas.range_m, 0, 0};  // Placeholder
}

Vector3 RelativeLocalizationFusion::computeCentroid(const std::vector<Vector3>& points) const {
    Vector3 centroid{0, 0, 0};
    if (points.empty()) return centroid;
    
    for (const auto& p : points) {
        centroid.x += p.x;
        centroid.y += p.y;
        centroid.z += p.z;
    }
    
    float n = static_cast<float>(points.size());
    centroid.x /= n;
    centroid.y /= n;
    centroid.z /= n;
    
    return centroid;
}

void RelativeLocalizationFusion::reset() {
    _uwb_measurements.clear();
    _vision_measurements.clear();
    _lidar_measurements.clear();
    _uwb_range_valid = false;
    _vision_valid = false;
    _lidar_valid = false;
}

} // namespace swarm::formation