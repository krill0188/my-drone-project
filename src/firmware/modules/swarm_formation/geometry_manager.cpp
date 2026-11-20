#include "swarm_formation.hpp"
#include <cmath>
#include <algorithm>

namespace swarm::formation {

FormationGeometryManager::FormationGeometryManager() : _current_geometry{}, _current_tolerances{} {}

bool FormationGeometryManager::setGeometry(const FormationGeometry& geometry) {
    // Validate geometry
    if (geometry.type == GeometryType::CUSTOM && geometry.relative_positions.empty()) {
        return false;  // Custom geometry requires positions
    }
    
    if (geometry.spacing_m <= 0.0f || geometry.spacing_m > 50.0f) {
        return false;  // Invalid spacing
    }
    
    _current_geometry = geometry;
    return true;
}

bool FormationGeometryManager::setTolerances(const FormationTolerances& tolerances) {
    if (tolerances.max_lateral_error_m <= 0.0f || tolerances.max_lateral_error_m > 5.0f) return false;
    if (tolerances.max_vertical_error_m <= 0.0f || tolerances.max_vertical_error_m > 5.0f) return false;
    if (tolerances.rejoin_timeout_s <= 0.0f || tolerances.rejoin_timeout_s > 30.0f) return false;
    if (tolerances.replan_propagation_ms <= 0.0f || tolerances.replan_propagation_ms > 1000.0f) return false;
    
    _current_tolerances = tolerances;
    return true;
}

std::vector<Vector3> FormationGeometryManager::computeOffsets(size_t num_followers) const {
    std::vector<Vector3> offsets;
    offsets.reserve(num_followers);
    
    if (num_followers == 0) return offsets;
    
    float spacing = _current_geometry.spacing_m;
    float alt_offset = _current_geometry.altitude_offset_m;
    
    switch (_current_geometry.type) {
        case GeometryType::LINE: {
            // Line formation behind leader (negative X in FLU)
            for (size_t i = 0; i < num_followers; ++i) {
                offsets.emplace_back(-static_cast<float>(i + 1) * spacing, 0.0f, alt_offset);
            }
            break;
        }
        case GeometryType::VEE: {
            // V formation: alternating left/right, increasing distance
            for (size_t i = 0; i < num_followers; ++i) {
                float side = (i % 2 == 0) ? -1.0f : 1.0f;
                float row = static_cast<float>(i / 2 + 1);
                offsets.emplace_back(-row * spacing, side * row * spacing * 0.707f, alt_offset);
            }
            break;
        }
        case GeometryType::DIAMOND: {
            // Diamond: 4 positions max
            static const float dx[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
            static const float dy[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
            size_t count = std::min(num_followers, size_t(4));
            for (size_t i = 0; i < count; ++i) {
                offsets.emplace_back(dx[i] * spacing, dy[i] * spacing, alt_offset);
            }
            // If more than 4, repeat pattern with increased spacing
            for (size_t i = 4; i < num_followers; ++i) {
                float scale = 1.0f + static_cast<float>(i / 4) * 1.5f;
                size_t base = i % 4;
                offsets.emplace_back(dx[base] * spacing * scale, dy[base] * spacing * scale, alt_offset);
            }
            break;
        }
        case GeometryType::COLUMN: {
            // Column: vertical stack behind leader
            for (size_t i = 0; i < num_followers; ++i) {
                offsets.emplace_back(-static_cast<float>(i + 1) * spacing, 0.0f, 
                                    alt_offset - static_cast<float>(i) * spacing * 0.5f);
            }
            break;
        }
        case GeometryType::CUSTOM: {
            // Use provided custom positions
            const auto& custom = _current_geometry.relative_positions;
            for (size_t i = 0; i < num_followers && i < custom.size(); ++i) {
                offsets.push_back(custom[i]);
            }
            // If more followers than custom positions, repeat last position with offset
            for (size_t i = custom.size(); i < num_followers; ++i) {
                Vector3 last = custom.empty() ? Vector3{0,0,0} : custom.back();
                offsets.emplace_back(last.x, last.y, last.z - static_cast<float>(i - custom.size() + 1) * spacing);
            }
            break;
        }
    }
    
    // Apply heading offset if specified
    if (_current_geometry.heading_offset_deg != 0.0f) {
        float rad = _current_geometry.heading_offset_deg * M_PI / 180.0f;
        float cos_h = std::cos(rad);
        float sin_h = std::sin(rad);
        for (auto& off : offsets) {
            float x = off.x * cos_h - off.y * sin_h;
            float y = off.x * sin_h + off.y * cos_h;
            off.x = x;
            off.y = y;
        }
    }
    
    return offsets;
}

FormationGeometry FormationGeometryManager::getCurrentGeometry() const {
    return _current_geometry;
}

FormationTolerances FormationGeometryManager::getCurrentTolerances() const {
    return _current_tolerances;
}

bool FormationGeometryManager::validateGeometry(const FormationGeometry& geometry) const {
    if (geometry.spacing_m <= 0.0f || geometry.spacing_m > 50.0f) return false;
    if (geometry.type == GeometryType::CUSTOM && geometry.relative_positions.empty()) return false;
    return true;
}

bool FormationGeometryManager::transitionGeometry(const FormationGeometry& new_geometry, float transition_time_s) {
    // In real implementation, would interpolate between current and new geometry over transition_time_s
    // For now, immediate transition
    return setGeometry(new_geometry);
}

} // namespace swarm::formation