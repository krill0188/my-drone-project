#ifndef INTENT_PARSER_HPP
#define INTENT_PARSER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <optional>
#include <chrono>
#include <Eigen/Dense>

namespace swarm::interface {

// Intent types
enum class IntentType : uint8_t {
    UNKNOWN = 0,
    // Critical (DistilBERT, < 10ms)
    EMERGENCY_STOP = 1,
    RETURN_HOME = 2,
    HOLD_POSITION = 3,
    LAND_NOW = 4,
    ABORT_MISSION = 5,
    // Mission (BERT, < 50ms)
    SEARCH_AREA = 10,
    FOLLOW_ME = 11,
    INSPECT_TARGET = 12,
    PATROL_ROUTE = 13,
    CHANGE_FORMATION = 14,
    GO_TO_WAYPOINT = 15,
    TAKEOFF = 16,
    LAND_AT = 17,
    // Complex (Cloud LLM)
    OPTIMIZE_COVERAGE = 20,
    PLAN_MISSION = 21,
    REPLAN_AVOID = 22,
    // System
    STATUS_REQUEST = 30,
    CANCEL = 99
};

enum class IntentSource : uint8_t {
    VOICE = 0,
    TABLET = 1,
    GESTURE = 2,
    AUTO = 3
};

enum class IntentPriority : uint8_t {
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
    CRITICAL = 3
};

struct IntentParameter {
    std::string key;
    std::string value;  // JSON string for complex params
};

struct ParsedIntent {
    IntentType type = IntentType::UNKNOWN;
    IntentSource source = IntentSource::VOICE;
    IntentPriority priority = IntentPriority::NORMAL;
    float confidence = 0.0f;  // 0.0 - 1.0
    std::string raw_text;
    std::string language = "en";  // ISO 639-1
    
    // Parameters (JSON for flexibility)
    std::map<std::string, std::string> parameters;
    
    // Metadata
    uint64_t timestamp_us = 0;
    std::string raw_audio_path;  // Optional: path to audio file
    float processing_time_ms = 0.0f;
    bool on_device = true;  // false = cloud processed
    
    // Validation
    bool valid() const {
        return type != IntentType::UNKNOWN && confidence > 0.5f;
    }
    
    // Convert to HumanIntentCmd message
    std::string to_json() const;
};

// Model inference result
struct InferenceResult {
    std::vector<float> logits;
    std::vector<std::pair<int, float>> top_k;  // class_id, prob
    float latency_ms = 0.0f;
};

// Intent parser configuration
struct IntentParserConfig {
    // Model paths
    std::string distilbert_model_path;  // DistilBERT-INT8 for critical
    std::string bert_model_path;        // BERT-base for mission
    std::string tokenizer_path;         // Tokenizer vocab
    
    // Runtime config
    std::string runtime = "tensorrt";  // tensorrt, onnxruntime
    bool enable_fp16 = true;
    bool enable_int8 = true;
    int max_batch_size = 1;
    int max_seq_length = 128;
    
    // Thresholds
    float critical_threshold = 0.9f;   // Critical intent threshold
    float mission_threshold = 0.75f;   // Mission intent threshold
    float unknown_threshold = 0.5f;    // Unknown/ignore threshold
    
    // Language support
    std::vector<std::string> supported_languages = {"en", "ko", "ja", "zh"};
    std::string default_language = "en";
    
    // Performance
    int max_latency_ms_critical = 10;
    int max_latency_ms_mission = 50;
    bool warmup_on_start = true;
    int warmup_iterations = 10;
    
    // Cloud fallback
    bool enable_cloud_fallback = false;
    std::string cloud_endpoint;
    int cloud_timeout_ms = 5000;
};

class IntentParser {
public:
    IntentParser();
    ~IntentParser();
    
    IntentParser(const IntentParser&) = delete;
    IntentParser& operator=(const IntentParser&) = delete;
    IntentParser(IntentParser&&) noexcept;
    IntentParser& operator=(IntentParser&&) noexcept;
    
    // Initialize parser
    bool initialize(const IntentParserConfig& config);
    
    // Parse text intent (from voice ASR, tablet text, etc.)
    ParsedIntent parseText(const std::string& text, 
                           IntentSource source = IntentSource::VOICE,
                           const std::string& language = "en");
    
    // Parse from audio (calls ASR internally or uses pre-transcribed text)
    ParsedIntent parseAudio(const std::string& audio_path,
                            IntentSource source = IntentSource::VOICE,
                            const std::string& language = "en");
    
    // Batch parse for multiple utterances
    std::vector<ParsedIntent> parseBatch(const std::vector<std::string>& texts,
                                         IntentSource source = IntentSource::VOICE,
                                         const std::string& language = "en");
    
    // Get model info
    struct ModelInfo {
        std::string critical_model_name;
        std::string mission_model_name;
        size_t critical_model_size_mb = 0;
        size_t mission_model_size_mb = 0;
        std::string runtime;
        bool critical_model_loaded = false;
        bool mission_model_loaded = false;
    };
    ModelInfo getModelInfo() const;
    
    // Get stats
    struct Stats {
        uint64_t total_parses = 0;
        uint64_t critical_parses = 0;
        uint64_t mission_parses = 0;
        uint64_t cloud_fallback_parses = 0;
        float avg_latency_ms = 0.0f;
        float avg_critical_latency_ms = 0.0f;
        float avg_mission_latency_ms = 0.0f;
        float avg_confidence = 0.0f;
    };
    const Stats& getStats() const;
    
    // Reload models (for A/B OTA updates)
    bool reloadModels(const IntentParserConfig& config);
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Set callback for parsed intents
    using IntentCallback = std::function<void(const ParsedIntent&)>;
    void setIntentCallback(IntentCallback cb) { intent_callback_ = cb; }
    
    // Set cloud callback
    using CloudCallback = std::function<std::optional<ParsedIntent>(const std::string&, const std::string&)>;
    void setCloudCallback(CloudCallback cb) { cloud_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    IntentParserConfig config_;
    bool initialized_ = false;
    
    std::function<void(const ParsedIntent&)> intent_callback_;
    std::function<std::optional<ParsedIntent>(const std::string&, const std::string&)> cloud_callback_;
};

}  // namespace swarm::interface

#endif  // INTENT_PARSER_HPP