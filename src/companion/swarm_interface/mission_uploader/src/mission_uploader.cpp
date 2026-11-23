#include "mission_uploader.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <chrono>
#include <random>
#include <algorithm>

namespace swarm::interface {

using json = nlohmann::json;

class MissionUploader::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    bool initialize(const MissionUploaderConfig& config) {
        config_ = config;
        
        // Load templates if path provided
        if (!config_.mission_templates_path.empty()) {
            loadTemplates(config_.mission_templates_path);
        }
        
        // Generate default templates if none loaded
        if (templates_.empty()) {
            generateDefaultTemplates();
        }
        
        initialized_ = true;
        return true;
    }
    
    bool loadTemplates(const std::string& path) {
        try {
            std::ifstream file(path);
            if (!file.is_open()) return false;
            
            json j;
            file >> j;
            
            for (const auto& [name, tmpl_json] : j.items()) {
                MissionTemplate tmpl;
                tmpl.name = name;
                tmpl.description = tmpl_json.value("description", "");
                tmpl.required_parameters = tmpl_json.value("required_parameters", std::vector<std::string>{});
                tmpl.default_parameters = tmpl_json.value("default_parameters", std::map<std::string, std::string>{});
                
                // Parse base mission (simplified)
                if (tmpl_json.contains("base_mission")) {
                    // Would parse full mission from JSON
                }
                
                templates_[name] = tmpl;
            }
            
            return true;
        } catch (const std::exception& e) {
            return false;
        }
    }
    
    void generateDefaultTemplates() {
        // Search area template
        {
            MissionTemplate tmpl;
            tmpl.name = "search_area";
            tmpl.description = "Lawnmower search pattern over rectangular area";
            tmpl.required_parameters = {"area", "pattern"};
            tmpl.default_parameters = {
                {"pattern", "lawnmower"},
                {"spacing_m", "20"},
                {"altitude_m", "50"},
                {"speed_mps", "10"}
            };
            templates_["search_area"] = tmpl;
        }
        
        // Follow me template
        {
            MissionTemplate tmpl;
            tmpl.name = "follow_me";
            tmpl.description = "Follow a target at specified distance";
            tmpl.required_parameters = {"target_id", "distance_m"};
            tmpl.default_parameters = {
                {"distance_m", "10"},
                {"altitude_m", "20"},
                {"max_speed_mps", "5"}
            };
            templates_["follow_me"] = tmpl;
        }
        
        // Inspect target template
        {
            MissionTemplate tmpl;
            tmpl.name = "inspect_target";
            tmpl.description = "Circle around target for inspection";
            tmpl.required_parameters = {"target"};
            tmpl.default_parameters = {
                {"radius_m", "15"},
                {"altitude_m", "30"},
                {"circles", "2"},
                {"speed_mps", "5"}
            };
            templates_["inspect_target"] = tmpl;
        }
        
        // Patrol route template
        {
            MissionTemplate tmpl;
            tmpl.name = "patrol_route";
            tmpl.description = "Follow waypoint route repeatedly";
            tmpl.required_parameters = {"waypoints"};
            tmpl.default_parameters = {
                {"repeat", "true"},
                {"altitude_m", "50"},
                {"speed_mps", "10"}
            };
            templates_["patrol_route"] = tmpl;
        }
        
        // Return to launch template
        {
            MissionTemplate tmpl;
            tmpl.name = "return_to_launch";
            tmpl.description = "Return to home position and land";
            tmpl.required_parameters = {};
            tmpl.default_parameters = {
                {"rtl_altitude_m", "50"},
                {"land_at_home", "true"}
            };
            templates_["return_to_launch"] = tmpl;
        }
        
        // Hold position template
        {
            MissionTemplate tmpl;
            tmpl.name = "hold_position";
            tmpl.description = "Hover at current position";
            tmpl.required_parameters = {};
            tmpl.default_parameters = {
                {"duration_s", "60"},
                {"altitude_m", "current"}
            };
            templates_["hold_position"] = tmpl;
        }
    }
    
    bool registerTemplate(const MissionTemplate& tmpl) {
        if (tmpl.name.empty()) return false;
        templates_[tmpl.name] = tmpl;
        return true;
    }
    
    std::vector<std::string> getTemplateNames() const {
        std::vector<std::string> names;
        for (const auto& [name, _] : templates_) {
            names.push_back(name);
        }
        return names;
    }
    
    std::optional<MissionTemplate> getTemplate(const std::string& name) const {
        auto it = templates_.find(name);
        if (it != templates_.end()) return it->second;
        return std::nullopt;
    }
    
    std::optional<swarm_msgs::msg::Mission> createMissionFromIntent(
        const swarm::interface::ParsedIntent& intent) {
        
        swarm_msgs::msg::Mission mission;
        mission.mission_id = generateMissionId();
        mission.timestamp_us = intent.timestamp_us;
        mission.created_by = "intent_parser";
        
        // Map intent type to template
        std::string template_name;
        switch (intent.type) {
            case swarm::interface::IntentType::SEARCH_AREA:
                template_name = "search_area";
                break;
            case swarm::interface::IntentType::FOLLOW_ME:
                template_name = "follow_me";
                break;
            case swarm::interface::IntentType::INSPECT_TARGET:
                template_name = "inspect_target";
                break;
            case swarm::interface::IntentType::PATROL_ROUTE:
                template_name = "patrol_route";
                break;
            case swarm::interface::IntentType::RETURN_HOME:
                template_name = "return_to_launch";
                break;
            case swarm::interface::IntentType::HOLD_POSITION:
                template_name = "hold_position";
                break;
            default:
                return std::nullopt;
        }
        
        auto tmpl_opt = getTemplate(intent.type);
        if (!tmpl_opt.has_value()) return std::nullopt;
        
        auto tmpl = tmpl_opt.value();
        
        // Build mission from template and intent parameters
        swarm_msgs::msg::Mission mission;
        mission.mission_id = generateMissionId();
        mission.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        mission.priority = static_cast<uint8_t>(intent.priority);
        
        // Set mission type
        mission.mission_type = static_cast<uint8_t>(intent.type);
        
        // Build tasks from intent parameters
        auto tasks = buildTasksFromIntent(intent);
        mission.tasks = tasks;
        mission.task_count = tasks.size();
        
        // Set constraints from intent parameters
        if (intent.parameters.count("area")) {
            mission.constraints.geofence.boundary = parseAreaToPolygon(
                intent.parameters["area"]);
        }
        
        if (intent.parameters.count("max_altitude_m")) {
            mission.constraints.max_altitude_m = 
                std::stof(intent.parameters["max_altitude_m"]);
        }
        
        if (intent.parameters.count("min_altitude_m")) {
            mission.constraints.min_altitude_m = 
                std::stof(intent.parameters["min_altitude_m"]);
        }
        
        if (intent.parameters.count("max_speed_mps")) {
            mission.constraints.max_speed_mps = 
                std::stof(intent.parameters["max_speed_mps"]);
        }
        
        // Set mission duration
        if (intent.parameters.count("duration_s")) {
            mission.max_duration_s = std::stof(intent.parameters["duration_s"]);
        } else {
            mission.max_duration_s = 3600.0f;  // Default 1 hour
        }
        
        return mission;
    }
    
    std::optional<swarm::interface::MissionTemplate> getTemplate(
        swarm::interface::IntentType type) {
        std::string template_name;
        switch (type) {
            case swarm::interface::IntentType::SEARCH_AREA: return getTemplate("search_area");
            case swarm::interface::IntentType::FOLLOW_ME: return getTemplate("follow_me");
            case swarm::interface::IntentType::INSPECT_TARGET: return getTemplate("inspect_target");
            case swarm::interface::IntentType::PATROL_ROUTE: return getTemplate("patrol_route");
            case swarm::interface::IntentType::RETURN_HOME: return getTemplate("return_to_launch");
            case swarm::interface::IntentType::HOLD_POSITION: return getTemplate("hold_position");
            default: return std::nullopt;
        }
    }
    
    std::vector<swarm_msgs::msg::Task> buildTasksFromIntent(
        const swarm::interface::ParsedIntent& intent) {
        
        std::vector<swarm_msgs::msg::Task> tasks;
        
        switch (intent.type) {
            case swarm::interface::IntentType::SEARCH_AREA: {
                auto task = createSearchTask(intent);
                if (task.has_value()) tasks.push_back(task.value());
                break;
            }
            case swarm::interface::IntentType::FOLLOW_ME: {
                auto task = createFollowTask(intent);
                if (task.has_value()) tasks.push_back(task.value());
                break;
            }
            case swarm::interface::IntentType::INSPECT_TARGET: {
                auto task = createInspectTask(intent);
                if (task.has_value()) tasks.push_back(task.value());
                break;
            }
            case swarm::interface::IntentType::PATROL_ROUTE: {
                auto tasks_opt = createPatrolTasks(intent);
                if (tasks_opt.has_value()) tasks = tasks_opt.value();
                break;
            }
            case swarm::interface::IntentType::RETURN_HOME: {
                auto task = createRTLTask(intent);
                if (task.has_value()) tasks.push_back(task.value());
                break;
            }
            case swarm::interface::IntentType::HOLD_POSITION: {
                auto task = createHoldTask(intent);
                if (task.has_value()) tasks.push_back(task.value());
                break;
            }
            default:
                break;
        }
        
        return tasks;
    }
    
    std::optional<swarm_msgs::msg::Task> createSearchTask(
        const swarm::interface::ParsedIntent& intent) {
        
        swarm_msgs::msg::Task task;
        task.task_id = generateTaskId();
        task.type = static_cast<uint8_t>(swarm::autonomy::TaskType::SEARCH);
        task.priority = static_cast<uint8_t>(swarm::interface::IntentPriority::HIGH);
        
        // Parse area
        std::string area = intent.parameters.count("area") ? intent.parameters["area"] : "default";
        task.geometry.type = 0;  // POLYGON
        task.geometry.polygon = parseAreaToPolygon(
            intent.parameters.count("area") ? intent.parameters["area"] : "default");
        
        // Search pattern
        std::string pattern = intent.parameters.count("pattern") ? 
            intent.parameters["pattern"] : "lawnmower";
        task.search_pattern = pattern;
        
        // Parameters
        float spacing = intent.parameters.count("spacing_m") ? 
            std::stof(intent.parameters["spacing_m"]) : 20.0f;
        task.parameters["spacing_m"] = std::to_string(spacing);
        
        float altitude = intent.parameters.count("altitude_m") ? 
            std::stof(intent.parameters["altitude_m"]) : 50.0f;
        task.parameters["altitude_m"] = std::to_string(altitude);
        
        float speed = intent.parameters.count("speed_mps") ? 
            std::stof(intent.parameters["speed_mps"]) : 10.0f;
        task.parameters["speed_mps"] = std::to_string(speed);
        
        // Success criteria
        task.success_criteria.coverage_threshold = 0.95;
        task.success_criteria.max_time_s = 3600.0f;
        
        return task;
    }
    
    std::optional<swarm_msgs::msg::Task> createFollowTask(
        const swarm::interface::ParsedIntent& intent) {
        
        swarm_msgs::msg::Task task;
        task.task_id = generateTaskId();
        task.type = static_cast<uint8_t>(swarm::autonomy::TaskType::FOLLOW);
        task.priority = static_cast<uint8_t>(swarm::interface::IntentPriority::HIGH);
        
        std::string target_id = intent.parameters.count("target_id") ? 
            intent.parameters["target_id"] : "0";
        task.parameters["target_id"] = target_id;
        
        float distance = intent.parameters.count("distance_m") ? 
            std::stof(intent.parameters["distance_m"]) : 10.0f;
        task.parameters["distance_m"] = std::to_string(distance);
        
        float altitude = intent.parameters.count("altitude_m") ? 
            std::stof(intent.parameters["altitude_m"]) : 20.0f;
        task.parameters["altitude_m"] = std::to_string(altitude);
        
        float max_speed = intent.parameters.count("max_speed_mps") ? 
            std::stof(intent.parameters["max_speed_mps"]) : 5.0f;
        task.parameters["max_speed_mps"] = std::to_string(max_speed);
        
        task.success_criteria.max_time_s = 3600.0f;
        
        return task;
    }
    
    std::optional<swarm_msgs::msg::Task> createInspectTask(
        const swarm::interface::ParsedIntent& intent) {
        
        swarm_msgs::msg::Task task;
        task.task_id = generateTaskId();
        task.type = static_cast<uint8_t>(swarm::autonomy::TaskType::INSPECT);
        task.priority = static_cast<uint8_t>(swarm::interface::IntentPriority::HIGH);
        
        std::string target = intent.parameters.count("target") ? 
            intent.parameters["target"] : "unknown";
        task.parameters["target"] = target;
        
        float radius = intent.parameters.count("radius_m") ? 
            std::stof(intent.parameters["radius_m"]) : 15.0f;
        task.parameters["radius_m"] = std::to_string(radius);
        
        float altitude = intent.parameters.count("altitude_m") ? 
            std::stof(intent.parameters["altitude_m"]) : 30.0f;
        task.parameters["altitude_m"] = std::to_string(altitude);
        
        int circles = intent.parameters.count("circles") ? 
            std::stoi(intent.parameters["circles"]) : 2;
        task.parameters["circles"] = std::to_string(circles);
        
        float speed = intent.parameters.count("speed_mps") ? 
            std::stof(intent.parameters["speed_mps"]) : 5.0f;
        task.parameters["speed_mps"] = std::to_string(speed);
        
        task.success_criteria.max_time_s = 1800.0f;
        
        return task;
    }
    
    std::optional<std::vector<swarm_msgs::msg::Task>> createPatrolTasks(
        const swarm::interface::ParsedIntent& intent) {
        
        std::vector<swarm_msgs::msg::Task> tasks;
        
        // Parse waypoints from parameters
        std::string waypoints_str = intent.parameters.count("waypoints") ? 
            intent.parameters["waypoints"] : "";
        
        if (waypoints_str.empty()) return std::nullopt;
        
        // Parse waypoints string (format: "x1,y1,z1;x2,y2,z2;...")
        auto waypoints = parseWaypoints(waypoints_str);
        if (waypoints.empty()) return std::nullopt;
        
        for (size_t i = 0; i < waypoints.size(); ++i) {
            swarm_msgs::msg::Task task;
            task.task_id = generateTaskId();
            task.type = static_cast<uint8_t>(swarm::autonomy::TaskType::WAYPOINT);
            task.priority = static_cast<uint8_t>(swarm::interface::IntentPriority::NORMAL);
            
            task.geometry.type = 1;  // POINT
            task.geometry.point = waypoints[i];
            
            task.parameters["waypoint_index"] = std::to_string(i);
            task.parameters["altitude_m"] = std::to_string(waypoints[i].z());
            
            float speed = intent.parameters.count("speed_mps") ? 
                std::stof(intent.parameters["speed_mps"]) : 10.0f;
            task.parameters["speed_mps"] = std::to_string(speed);
            
            // Dependencies - sequential
            if (i > 0) {
                // Would add dependency on previous task
            }
            
            tasks.push_back(task);
        }
        
        if (intent.parameters.count("repeat") && 
            intent.parameters["repeat"] == "true") {
            // Add dependency from last to first for looping
        }
        
        return tasks;
    }
    
    std::optional<swarm_msgs::msg::Task> createRTLTask(
        const swarm::interface::ParsedIntent& intent) {
        
        swarm_msgs::msg::Task task;
        task.task_id = generateTaskId();
        task.type = static_cast<uint8_t>(swarm::autonomy::TaskType::RTL);
        task.priority = static_cast<uint8_t>(swarm::interface::IntentPriority::CRITICAL);
        
        float rtl_altitude = intent.parameters.count("rtl_altitude_m") ? 
            std::stof(intent.parameters["rtl_altitude_m"]) : 50.0f;
        task.parameters["rtl_altitude_m"] = std::to_string(rtl_altitude);
        
        bool land_at_home = intent.parameters.count("land_at_home") ? 
            (intent.parameters["land_at_home"] == "true") : true;
        task.parameters["land_at_home"] = land_at_home ? "true" : "false";
        
        task.success_criteria.max_time_s = 600.0f;
        
        return task;
    }
    
    std::optional<swarm_msgs::msg::Task> createHoldTask(
        const swarm::interface::ParsedIntent& intent) {
        
        swarm_msgs::msg::Task task;
        task.task_id = generateTaskId();
        task.type = static_cast<uint8_t>(swarm::autonomy::TaskType::HOVER);
        task.priority = static_cast<uint8_t>(swarm::interface::IntentPriority::NORMAL);
        
        float duration = intent.parameters.count("duration_s") ? 
            std::stof(intent.parameters["duration_s"]) : 60.0f;
        task.parameters["duration_s"] = std::to_string(duration);
        
        std::string altitude = intent.parameters.count("altitude_m") ? 
            intent.parameters["altitude_m"] : "current";
        task.parameters["altitude_m"] = altitude;
        
        task.success_criteria.max_time_s = duration + 10.0f;
        
        return task;
    }
    
    // Helper functions
    std::string generateMissionId() {
        auto now = std::chrono::system_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(1000, 9999);
        return "M_" + std::to_string(us) + "_" + std::to_string(dis(rd));
    }
    
    std::string generateTaskId() {
        auto now = std::chrono::system_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(1000, 9999);
        return "T_" + std::to_string(us) + "_" + std::to_string(dis(rd));
    }
    
    std::vector<Eigen::Vector3f> parseWaypoints(const std::string& waypoints_str) {
        std::vector<Eigen::Vector3f> waypoints;
        // Simple parsing: "x1,y1,z1;x2,y2,z2;..."
        size_t start = 0;
        while (start < waypoints_str.size()) {
            size_t end = waypoints_str.find(';', start);
            if (end == std::string::npos) end = waypoints_str.size();
            
            std::string wp_str = waypoints_str.substr(start, end - start);
            std::vector<std::string> coords;
            size_t coord_start = 0;
            while (coord_start < wp_str.size()) {
                size_t coord_end = wp_str.find(',', coord_start);
                if (coord_end == std::string::npos) coord_end = wp_str.size();
                coords.push_back(wp_str.substr(coord_start, coord_end - coord_start));
                coord_start = coord_end + 1;
            }
            
            if (coords.size() >= 3) {
                Eigen::Vector3f wp(
                    std::stof(coords[0]),
                    std::stof(coords[1]),
                    std::stof(coords[2])
                );
                waypoints.push_back(wp);
            }
            
            start = end + 1;
        }
        return waypoints;
    }
    
    std::vector<Eigen::Vector3f> parseAreaToPolygon(const std::string& area) {
        // Simplified: would parse named area or coordinates
        // For now, return default square
        return {
            {-100, -100, 0}, {100, -100, 0}, {100, 100, 0}, {-100, 100, 0}
        };
    }
};

MissionUploader::MissionUploader() : pimpl_(std::make_unique<Impl>()) {}
MissionUploader::~MissionUploader() = default;
MissionUploader::MissionUploader(MissionUploader&&) noexcept = default;
MissionUploader& MissionUploader::operator=(MissionUploader&&) noexcept = default;

bool MissionUploader::initialize(const MissionUploaderConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

bool MissionUploader::loadTemplates(const std::string& path) {
    return pimpl_->loadTemplates(path);
}

bool MissionUploader::registerTemplate(const MissionTemplate& template_) {
    return pimpl_->registerTemplate(template_);
}

std::vector<std::string> MissionUploader::getTemplateNames() const {
    return pimpl_->getTemplateNames();
}

std::optional<MissionTemplate> MissionUploader::getTemplate(const std::string& name) const {
    return pimpl_->getTemplate(name);
}

std::optional<swarm_msgs::msg::Mission> MissionUploader::createMissionFromIntent(
    const swarm::interface::ParsedIntent& intent) {
    return pimpl_->createMissionFromIntent(intent);
}

UploadResult MissionUploader::uploadMission(const swarm_msgs::msg::Mission& mission,
                                            const std::vector<uint16_t>& target_drones) {
    // Implementation would send mission to target drones via mesh
    UploadResult result;
    result.success = true;
    result.mission_id = mission.mission_id;
    result.upload_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    for (uint16_t id : target_drones) {
        result.drone_acks[id] = false;
    }
    
    return result;
}

UploadResult MissionUploader::uploadFromIntent(const swarm::interface::ParsedIntent& intent,
                                               const std::vector<uint16_t>& target_drones) {
    auto mission_opt = createMissionFromIntent(intent);
    if (!mission_opt.has_value()) {
        UploadResult result;
        result.success = false;
        result.error_message = "Failed to create mission from intent";
        return result;
    }
    
    return uploadMission(mission_opt.value(), target_drones);
}

bool MissionUploader::validateMission(const swarm_msgs::msg::Mission& mission,
                                      std::string& error_message) const {
    // Check mission ID
    if (mission.mission_id.empty()) {
        error_message = "Mission ID is empty";
        return false;
    }
    
    // Check tasks
    if (mission.tasks.empty()) {
        error_message = "Mission has no tasks";
        return false;
    }
    
    // Check task count
    if (mission.task_count != mission.tasks.size()) {
        error_message = "Task count mismatch";
        return false;
    }
    
    // Check constraints
    if (mission.constraints.min_altitude_m > mission.constraints.max_altitude_m) {
        error_message = "Invalid altitude constraints";
        return false;
    }
    
    // Check duration
    if (mission.max_duration_s > 3600.0f) {
        error_message = "Mission duration exceeds maximum";
        return false;
    }
    
    return true;
}

bool MissionUploader::cancelMission(const std::string& mission_id) {
    // Would send cancellation to drones
    return true;
}

std::optional<MissionUploadStatus> MissionUploader::getUploadStatus(
    const std::string& mission_id) const {
    return std::nullopt;  // Would track active uploads
}

std::vector<MissionUploadStatus> MissionUploader::getAllUploadStatuses() const {
    return {};
}

void MissionUploader::setUploadMissionCallback(UploadMissionCallback cb) {
    upload_complete_callback_ = cb;
}

void MissionUploader::setValidateMissionCallback(ValidateMissionCallback cb) {
    validate_mission_callback_ = cb;
}

void MissionUploader::setUploadCompleteCallback(UploadCompleteCallback cb) {
    upload_complete_callback_ = cb;
}

const MissionUploaderConfig& MissionUploader::getConfig() const {
    return config_;
}

bool MissionUploader::isInitialized() const {
    return initialized_;
}

}  // namespace swarm::interface