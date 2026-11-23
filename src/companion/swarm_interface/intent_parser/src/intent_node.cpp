#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "swarm_msgs/msg/human_intent_cmd.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_interface_intent_parser/intent_parser.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class IntentNode : public rclcpp::Node {
public:
    IntentNode() : Node("intent_parser_node") {
        declareParameters();
        loadParameters();
        
        // Initialize intent parser
        intent_parser_ = std::make_unique<swarm::interface::IntentParser>();
        swarm::interface::IntentParserConfig parser_config;
        
        parser_config.distilbert_model_path = get_parameter("distilbert_model_path").as_string();
        parser_config.bert_model_path = get_parameter("bert_model_path").as_string();
        parser_config.tokenizer_path = get_parameter("tokenizer_path").as_string();
        parser_config.runtime = get_parameter("runtime").as_string();
        parser_config.enable_fp16 = get_parameter("enable_fp16").as_bool();
        parser_config.enable_int8 = get_parameter("enable_int8").as_bool();
        parser_config.critical_threshold = get_parameter("critical_threshold").as_double();
        parser_config.mission_threshold = get_parameter("mission_threshold").as_double();
        parser_config.unknown_threshold = get_parameter("unknown_threshold").as_double();
        parser_config.default_language = get_parameter("default_language").as_string();
        parser_config.max_latency_ms_critical = get_parameter("max_latency_ms_critical").as_int();
        parser_config.max_latency_ms_mission = get_parameter("max_latency_ms_mission").as_int();
        parser_config.warmup_on_start = get_parameter("warmup_on_start").as_bool();
        parser_config.warmup_iterations = get_parameter("warmup_iterations").as_int();
        parser_config.enable_cloud_fallback = get_parameter("enable_cloud_fallback").as_bool();
        parser_config.cloud_endpoint = get_parameter("cloud_endpoint").as_string();
        parser_config.cloud_timeout_ms = get_parameter("cloud_timeout_ms").as_int();
        
        if (!intent_parser_->initialize(parser_config)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize intent parser");
        }
        
        // Setup subscriptions
        setupSubscriptions();
        
        // Setup publishers
        setupPublishers();
        
        // Setup services
        setupServices();
        
        // Set callbacks
        intent_parser_->setIntentCallback(
            std::bind(&IntentNode::onIntentParsed, this, std::placeholders::_1));
        
        intent_parser_->setCloudCallback(
            std::bind(&IntentNode::onCloudFallback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timer for stats
        stats_timer_ = create_wall_timer(
            10s, std::bind(&IntentNode::publishStats, this));
        
        RCLCPP_INFO(get_logger(), "Intent Parser Node initialized");
    }
    
private:
    // Configuration
    void declareParameters() {
        declare_parameter("distilbert_model_path", "/models/distilbert_intent_critical.engine");
        declare_parameter("bert_model_path", "/models/bert_intent_mission.engine");
        declare_parameter("tokenizer_path", "/models/tokenizer/vocab.txt");
        declare_parameter("runtime", "tensorrt");
        declare_parameter("enable_fp16", true);
        declare_parameter("enable_int8", true);
        declare_parameter("critical_threshold", 0.9);
        declare_parameter("mission_threshold", 0.75);
        declare_parameter("unknown_threshold", 0.5);
        declare_parameter("default_language", "en");
        declare_parameter("max_latency_ms_critical", 10);
        declare_parameter("max_latency_ms_mission", 50);
        declare_parameter("warmup_on_start", true);
        declare_parameter("warmup_iterations", 10);
        declare_parameter("enable_cloud_fallback", false);
        declare_parameter("cloud_endpoint", "");
        declare_parameter("cloud_timeout_ms", 5000);
        declare_parameter("drone_id", 1);
        declare_parameter("publish_raw_intent", false);
    }
    
    void loadParameters() {
        drone_id_ = get_parameter("drone_id").as_int();
        publish_raw_intent_ = get_parameter("publish_raw_intent").as_bool();
    }
    
    void setupSubscriptions() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
        // Voice command input (from ASR node)
        voice_sub_ = create_subscription<std_msgs::msg::String>(
            "/drone" + std::to_string(drone_id_) + "/voice_command", qos,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                processTextInput(msg->data, swarm::interface::IntentSource::VOICE);
            });
        
        // Tablet/text input (from ground station or tablet app)
        tablet_sub_ = create_subscription<std_msgs::msg::String>(
            "/drone" + std::to_string(drone_id_) + "/tablet_command", qos,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                processTextInput(msg->data, swarm::interface::IntentSource::TABLET);
            });
        
        // Gesture input (from gesture recognition node)
        gesture_sub_ = create_subscription<std_msgs::msg::String>(
            "/drone" + std::to_string(drone_id_) + "/gesture_command", qos,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                processTextInput(msg->data, swarm::interface::IntentSource::GESTURE);
            });
        
        // Auto-generated commands (from safety monitor, etc.)
        auto_sub_ = create_subscription<std_msgs::msg::String>(
            "/drone" + std::to_string(drone_id_) + "/auto_command", qos,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                processTextInput(msg->data, swarm::interface::IntentSource::AUTO);
            });
        
        // Agent state for context enrichment
        agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
            "/swarm/agent_states", rclcpp::QoS(10).best_effort(),
            [this](const swarm_msgs::msg::AgentState::SharedPtr msg) {
                if (msg->sysid == drone_id_) {
                    updateAgentContext(msg);
                }
            });
    }
    
    void setupPublishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        
        // Human intent command (main output)
        intent_pub_ = create_publisher<swarm_msgs::msg::HumanIntentCmd>(
            "/drone" + std::to_string(drone_id_) + "/human_intent_cmd", qos);
        
        // Raw intent for debugging
        if (publish_raw_intent_) {
            raw_intent_pub_ = create_publisher<std_msgs::msg::String>(
                "/drone" + std::to_string(drone_id_) + "/raw_intent", 
                rclcpp::QoS(10).reliable());
        }
        
        // Stats
        stats_pub_ = create_publisher<swarm_msgs::msg::IntentParserStats>(
            "/drone" + std::to_string(drone_id_) + "/intent_parser_stats", qos);
    }
    
    void setupServices() {
        // Parse text service
        parse_service_ = create_service<swarm_msgs::srv::ParseIntent>(
            "/drone" + std::to_string(drone_id_) + "/parse_intent",
            [this](const std::shared_ptr<rmw_request_id_t>,
                   const std::shared_ptr<swarm_msgs::srv::ParseIntent::Request> request,
                   std::shared_ptr<swarm_msgs::srv::ParseIntent::Response> response) {
                auto intent = intent_parser_->parseText(request->text, 
                                                        static_cast<swarm::interface::IntentSource>(request->source),
                                                        request->language);
                fillResponse(response, intent);
            });
    }
    
    void updateAgentContext(const swarm_msgs::msg::AgentState::SharedPtr msg) {
        agent_context_ = *msg;
        has_agent_context_ = true;
    }
    
    void processTextInput(const std::string& text, swarm::interface::IntentSource source) {
        if (text.empty()) return;
        
        auto intent = intent_parser_->parseText(text, source, default_language_);
        
        if (intent.valid()) {
            // Publish raw intent for debugging if enabled
            if (publish_raw_intent_ && raw_intent_pub_) {
                auto raw_msg = std_msgs::msg::String();
                raw_msg.data = intent.to_json();
                raw_intent_pub_->publish(raw_msg);
            }
        } else {
            RCLCPP_WARN(get_logger(), "Failed to parse intent (confidence: %.2f): %s", 
                       intent.confidence, text.c_str());
        }
    }
    
    void updateAgentContext(const swarm_msgs::msg::AgentState::SharedPtr msg) {
        agent_context_ = *msg;
        has_agent_context_ = true;
    }
    
    void onIntentParsed(const swarm::interface::ParsedIntent& intent) {
        // Add agent context to parameters
        swarm::interface::ParsedIntent enriched_intent = intent;
        if (has_agent_context_) {
            enriched_intent.parameters["agent_position_x"] = std::to_string(agent_context_.position.x);
            enriched_intent.parameters["agent_position_y"] = std::to_string(agent_context_.position.y);
            enriched_intent.parameters["agent_position_z"] = std::to_string(agent_context_.position.z);
            enriched_intent.parameters["agent_velocity_x"] = std::to_string(agent_context_.velocity.x);
            enriched_intent.parameters["agent_velocity_y"] = std::to_string(agent_context_.velocity.y);
            enriched_intent.parameters["agent_velocity_z"] = std::to_string(agent_context_.velocity.z);
            enriched_intent.parameters["agent_battery_pct"] = std::to_string(agent_context_.battery_percentage);
            enriched_intent.parameters["agent_mode"] = std::to_string(agent_context_.mode);
        }
        
        // Add source-specific context
        enriched_intent.parameters["source"] = intentSourceToString(intent.source);
        enriched_intent.parameters["drone_id"] = std::to_string(drone_id_);
        
        // Publish to HumanIntentCmd topic
        auto msg = swarm_msgs::msg::HumanIntentCmd();
        msg.timestamp_us = intent.timestamp_us;
        msg.drone_id = drone_id_;
        msg.source_id = 0;  // Ground station or self
        msg.intent_type = static_cast<uint8_t>(intent.type);
        msg.command = intent.raw_text;
        msg.confidence = intent.confidence;
        msg.language = intent.language;
        msg.parameters = intent.parameters.empty() ? "{}" : mapToJson(intent.parameters);
        msg.priority = static_cast<uint8_t>(intent.priority);
        
        intent_pub_->publish(msg);
        
        RCLCPP_INFO(get_logger(), "Intent parsed: type=%d, cmd='%s', conf=%.2f%%, src=%d, pri=%d",
                    static_cast<int>(intent.type), intent.raw_text.c_str(), 
                    intent.confidence * 100, static_cast<int>(intent.source),
                    static_cast<int>(intent.priority));
    }
    
    std::optional<swarm::interface::ParsedIntent> onCloudFallback(const std::string& text, 
                                                                   const std::string& language) {
        // Cloud fallback implementation
        // Would call cloud API (Llama/GPT) for complex queries
        return std::nullopt;
    }
    
    void fillResponse(std::shared_ptr<swarm_msgs::srv::ParseIntent::Response> response,
                      const swarm::interface::ParsedIntent& intent) {
        response->success = intent.valid();
        response->intent_type = static_cast<uint8_t>(intent.type);
        response->confidence = intent.confidence;
        response->command = intent.raw_text;
        response->parameters = intent.parameters.empty() ? "{}" : mapToJson(intent.parameters);
        response->priority = static_cast<uint8_t>(intent.priority);
        response->processing_time_ms = intent.processing_time_ms;
        response->on_device = intent.on_device;
    }
    
    void publishStats() {
        auto stats = intent_parser_->getStats();
        
        auto msg = swarm_msgs::msg::IntentParserStats();
        msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.drone_id = drone_id_;
        msg.total_parses = stats.total_parses;
        msg.critical_parses = stats.critical_parses;
        msg.mission_parses = stats.mission_parses;
        msg.cloud_fallback_parses = stats.cloud_fallback_parses;
        msg.avg_latency_ms = stats.avg_latency_ms;
        msg.avg_critical_latency_ms = stats.avg_critical_latency_ms;
        msg.avg_mission_latency_ms = stats.avg_mission_latency_ms;
        msg.avg_confidence = stats.avg_confidence;
        
        stats_pub_->publish(msg);
    }
    
    std::string mapToJson(const std::map<std::string, std::string>& m) {
        nlohmann::json j = m;
        return j.dump();
    }
    
    std::string intentSourceToString(swarm::interface::IntentSource source) {
        switch (source) {
            case swarm::interface::IntentSource::VOICE: return "voice";
            case swarm::interface::IntentSource::TABLET: return "tablet";
            case swarm::interface::IntentSource::GESTURE: return "gesture";
            case swarm::interface::IntentSource::AUTO: return "auto";
            default: return "unknown";
        }
    }
    
    // Members
    std::unique_ptr<swarm::interface::IntentParser> intent_parser_;
    uint16_t drone_id_ = 1;
    std::string default_language_ = "en";
    bool publish_raw_intent_ = false;
    bool has_agent_context_ = false;
    swarm_msgs::msg::AgentState agent_context_;
    
    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tablet_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gesture_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr auto_sub_;
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    
    rclcpp::Publisher<swarm_msgs::msg::HumanIntentCmd>::SharedPtr intent_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_intent_pub_;
    rclcpp::Publisher<swarm_msgs::msg::IntentParserStats>::SharedPtr stats_pub_;
    rclcpp::Service<swarm_msgs::srv::ParseIntent>::SharedPtr parse_service_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}