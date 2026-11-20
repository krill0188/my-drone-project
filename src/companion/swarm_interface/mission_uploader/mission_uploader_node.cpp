#include "rclcpp/rclcpp.hpp"
#include "swarm_interface/mission_uploader/mission_uploader.hpp"
#include "swarm_autonomy/trajectory_planner/trajectory_planner.hpp"
#include "swarm_autonomy/trajectory_planner/formation_trajectory_planner.hpp"
#include <memory>
#include <string>

namespace swarm::interface {

class MissionUploaderNode : public rclcpp::Node {
public:
    MissionUploaderNode() : Node("mission_uploader_node") {
        // Parameters
        std::string mission_file = declare_parameter("mission_file", "");
        std::string formation_file = declare_parameter("formation_file", "");
        bool auto_upload = declare_parameter("auto_upload", true);
        
        // Initialize planners
        planner::PlannerConfig config;
        config.max_velocity = declare_parameter("max_velocity", 10.0);
        config.max_acceleration = declare_parameter("max_acceleration", 5.0);
        config.resolution = declare_parameter("resolution", 0.5);
        config.safety_margin = declare_parameter("safety_margin", 2.0);
        config.mpc_horizon = declare_parameter("mpc_horizon", 20);
        config.mpc_dt = declare_parameter("mpc_dt", 0.1);
        
        _trajectory_planner = std::make_shared<planner::TrajectoryPlanner>(config);
        _formation_planner = std::make_shared<planner::FormationTrajectoryPlanner>(config);
        
        // Initialize mission uploader
        _mission_uploader = std::make_unique<MissionUploader>(
            shared_from_this(), _trajectory_planner, _formation_planner);
        
        // Set callback
        _mission_uploader->setMissionCallback(
            [this](const Mission& mission, bool success, const std::string& message) {
                if (success) {
                    RCLCPP_INFO(get_logger(), "Mission '%s' uploaded successfully: %s", 
                               mission.name.c_str(), message.c_str());
                } else {
                    RCLCPP_ERROR(get_logger(), "Mission upload failed: %s", message.c_str());
                }
            });
        
        // Load mission and formation files if provided
        if (!mission_file.empty() && !formation_file.empty()) {
            loadMissionAndFormation(mission_file, formation_file);
        }
        
        // Auto-upload if configured
        if (auto_upload && _mission_loaded && _formation_loaded) {
            _timer = create_wall_timer(
                std::chrono::seconds(2),
                [this]() {
                    _timer->cancel();
                    uploadMission();
                });
        }
        
        RCLCPP_INFO(get_logger(), "Mission Uploader Node initialized");
    }
    
    void loadMissionAndFormation(const std::string& mission_file, const std::string& formation_file) {
        auto mission = loadMissionFromYaml(mission_file);
        auto formation = loadFormationFromYaml(formation_file);
        
        if (mission && formation) {
            _loaded_mission = *mission;
            _loaded_formation = *formation;
            _mission_loaded = true;
            _formation_loaded = true;
            RCLCPP_INFO(get_logger(), "Loaded mission: %s, formation: %s", 
                       _loaded_mission.name.c_str(), 
                       formation_file.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to load mission or formation file");
        }
    }
    
    void uploadMission() {
        if (!_mission_loaded || !_formation_loaded) {
            RCLCPP_ERROR(get_logger(), "Mission or formation not loaded");
            return;
        }
        
        _mission_uploader->uploadMissionAsync(
            _loaded_mission, _loaded_formation,
            [this](const Mission& mission, bool success, const std::string& message) {
                if (success) {
                    RCLCPP_INFO(get_logger(), "Auto-upload completed successfully");
                }
            });
    }

private:
    std::shared_ptr<planner::TrajectoryPlanner> _trajectory_planner;
    std::shared_ptr<planner::FormationTrajectoryPlanner> _formation_planner;
    std::unique_ptr<MissionUploader> _mission_uploader;
    Mission _loaded_mission;
    FormationConfig _loaded_formation;
    bool _mission_loaded = false;
    bool _formation_loaded = false;
    rclcpp::TimerBase::SharedPtr _timer;
};

} // namespace swarm::interface

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<swarm::interface::MissionUploaderNode>());
    rclcpp::shutdown();
    return 0;
}