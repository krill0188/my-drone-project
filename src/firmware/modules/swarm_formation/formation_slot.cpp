#include "swarm_formation.hpp"
#include <algorithm>
#include <cmath>

namespace swarm::formation {

FormationSlotManager::FormationSlotManager() : _geometry_type(GeometryType::VEE), _spacing(5.0f) {}

bool FormationSlotManager::initialize(const FormationGeometry& geometry, const std::vector<uint16_t>& follower_ids) {
    _geometry_type = geometry.type;
    _spacing = geometry.spacing_m;
    _altitude_offset = geometry.altitude_offset_m;
    _heading_offset = geometry.heading_offset_deg;
    
    _slots.clear();
    _slots.reserve(follower_ids.size());
    
    // Assign slots based on geometry
    auto offsets = computeOffsets(follower_ids.size());
    
    for (size_t i = 0; i < follower_ids.size() && i < offsets.size(); ++i) {
        FormationSlot slot;
        slot.follower_id = follower_ids[i];
        slot.offset_flu = offsets[i];
        slot.gain_xy = 1.0f;
        slot.gain_z = 1.0f;
        slot.gain_yaw = 1.0f;
        _slots.push_back(slot);
    }
    
    return true;
}

std::vector<Vector3> FormationSlotManager::computeOffsets(size_t num_followers) const {
    std::vector<Vector3> offsets;
    offsets.reserve(num_followers);
    
    if (num_followers == 0) return offsets;
    
    float spacing = _spacing;
    float alt = _altitude_offset;
    
    switch (_geometry_type) {
        case GeometryType::LINE: {
            for (size_t i = 0; i < num_followers; ++i) {
                offsets.emplace_back(-static_cast<float>(i + 1) * spacing, 0.0f, alt);
            }
            break;
        }
        case GeometryType::VEE: {
            for (size_t i = 0; i < num_followers; ++i) {
                float side = (i % 2 == 0) ? -1.0f : 1.0f;
                float row = static_cast<float>(i / 2 + 1);
                offsets.emplace_back(-row * spacing, side * row * spacing * 0.707f, alt);
            }
            break;
        }
        case GeometryType::DIAMOND: {
            static const float dx[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
            static const float dy[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
            size_t count = std::min(num_followers, size_t(4));
            for (size_t i = 0; i < count; ++i) {
                offsets.emplace_back(dx[i] * _spacing, dy[i] * _spacing, _altitude_offset);
            }
            for (size_t i = 4; i < num_followers; ++i) {
                float scale = 1.0f + static_cast<float>(i / 4) * 1.5f;
                size_t base = i % 4;
                offsets.emplace_back(dx[base] * _spacing * scale, dy[base] * _spacing * scale, _altitude_offset);
            }
            break;
        }
        case GeometryType::COLUMN: {
            for (size_t i = 0; i < num_followers; ++i) {
                offsets.emplace_back(-static_cast<float>(i + 1) * _spacing, 0.0f, 
                                    _altitude_offset - static_cast<float>(i) * _spacing * 0.5f);
            }
            break;
        }
        case GeometryType::CUSTOM: {
            // For custom, would use pre-configured positions
            // Placeholder: line formation
            for (size_t i = 0; i < num_followers; ++i) {
                offsets.emplace_back(-static_cast<float>(i + 1) * _spacing, 0.0f, _altitude_offset);
            }
            break;
        }
    }
    
    // Apply heading offset
    if (_heading_offset != 0.0f) {
        float rad = _heading_offset * M_PI / 180.0f;
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

bool FormationSlotManager::assignSlot(uint16_t follower_id, const FormationSlot& slot) {
    // Check if follower already has a slot
    auto it = std::find_if(_slots.begin(), _slots.end(),
        [follower_id](const FormationSlot& s) { return s.follower_id == follower_id; });
    
    if (it != _slots.end()) {
        *it = slot;
    } else {
        _slots.push_back(slot);
    }
    return true;
}

bool FormationSlotManager::removeSlot(uint16_t follower_id) {
    auto it = std::find_if(_slots.begin(), _slots.end(),
        [follower_id](const FormationSlot& s) { return s.follower_id == follower_id; });
    
    if (it != _slots.end()) {
        _slots.erase(it);
        return true;
    }
    return false;
}

std::optional<FormationSlot> FormationSlotManager::getSlot(uint16_t follower_id) const {
    auto it = std::find_if(_slots.begin(), _slots.end(),
        [follower_id](const FormationSlot& s) { return s.follower_id == follower_id; });
    
    if (it != _slots.end()) {
        return *it;
    }
    return std::nullopt;
}

std::vector<FormationSlot> FormationSlotManager::getAllSlots() const {
    return _slots;
}

bool FormationSlotManager::reassignSlots(const std::vector<uint16_t>& new_follower_ids) {
    // Recompute slots for new set of followers
    std::vector<FormationSlot> new_slots;
    new_slots.reserve(new_follower_ids.size());
    
    auto offsets = computeOffsets(new_follower_ids.size());
    
    for (size_t i = 0; i < new_follower_ids.size() && i < offsets.size(); ++i) {
        FormationSlot slot;
        slot.follower_id = new_follower_ids[i];
        slot.offset_flu = offsets[i];
        
        // Try to preserve gains from old assignment
        auto old_slot = getSlot(new_follower_ids[i]);
        if (old_slot) {
            slot.gain_xy = old_slot->gain_xy;
            slot.gain_z = old_slot->gain_z;
            slot.gain_yaw = old_slot->gain_yaw;
        } else {
            slot.gain_xy = 1.0f;
            slot.gain_z = 1.0f;
            slot.gain_yaw = 1.0f;
        }
        new_slots.push_back(slot);
    }
    
    _slots = std::move(new_slots);
    return true;
}

bool FormationSlotManager::updateGeometry(const FormationGeometry& geometry) {
    _geometry_type = geometry.type;
    _spacing = geometry.spacing_m;
    _altitude_offset = geometry.altitude_offset_m;
    _heading_offset = geometry.heading_offset_deg;
    
    // Recompute offsets for existing followers
    std::vector<uint16_t> follower_ids;
    for (const auto& slot : _slots) {
        follower_ids.push_back(slot.follower_id);
    }
    
    auto offsets = computeOffsets(follower_ids.size());
    for (size_t i = 0; i < _slots.size() && i < offsets.size(); ++i) {
        _slots[i].offset_flu = offsets[i];
    }
    
    return true;
}

bool FormationSlotManager::setGains(uint16_t follower_id, float gain_xy, float gain_z, float gain_yaw) {
    auto it = std::find_if(_slots.begin(), _slots.end(),
        [follower_id](const FormationSlot& s) { return s.follower_id == follower_id; });
    
    if (it != _slots.end()) {
        it->gain_xy = gain_xy;
        it->gain_z = gain_z;
        it->gain_yaw = gain_yaw;
        return true;
    }
    return false;
}

bool FormationSlotManager::autoTuneGains(uint16_t follower_id, const Vector3& position_error, const Vector3& velocity_error) {
    // Simple adaptive gain tuning based on error
    auto it = std::find_if(_slots.begin(), _slots.end(),
        [follower_id](const FormationSlot& s) { return s.follower_id == follower_id; });
    
    if (it == _slots.end()) return false;
    
    float pos_error_mag = std::sqrt(position_error.x * position_error.x + 
                                     position_error.y * position_error.y + 
                                     position_error.z * position_error.z);
    float vel_error_mag = std::sqrt(velocity_error.x * velocity_error.x + 
                                     velocity_error.y * velocity_error.y + 
                                     velocity_error.z * velocity_error.z);
    
    // Adjust gains based on error magnitude
    if (pos_error_mag > 1.0f) {
        it->gain_xy = std::min(it->gain_xy * 1.1f, 2.0f);
        it->gain_z = std::min(it->gain_z * 1.1f, 2.0f);
    } else if (pos_error_mag < 0.1f && vel_error_mag < 0.1f) {
        it->gain_xy = std::max(it->gain_xy * 0.95f, 0.5f);
        it->gain_z = std::max(it->gain_z * 0.95f, 0.5f);
    }
    
    return true;
}

} // namespace swarm::formation