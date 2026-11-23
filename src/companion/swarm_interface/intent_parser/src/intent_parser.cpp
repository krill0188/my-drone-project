#include "intent_parser.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <thread>
#include <mutex>

namespace swarm::interface {

using json = nlohmann::json;

class IntentParser::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    bool initialize(const IntentParserConfig& config) {
        config_ = config;
        
        // Load tokenizer
        if (!loadTokenizer(config.tokenizer_path)) {
            return false;
        }
        
        // Load critical model (DistilBERT)
        if (!loadCriticalModel(config.distilbert_model_path)) {
            std::cerr << "Warning: Failed to load critical model" << std::endl;
        }
        
        // Load mission model (BERT)
        if (!loadMissionModel(config.bert_model_path)) {
            std::cerr << "Warning: Failed to load mission model" << std::endl;
        }
        
        // Warmup
        if (config.warmup_on_start) {
            warmup(config.warmup_iterations);
        }
        
        return critical_model_loaded_ || mission_model_loaded_;
    }
    
    ParsedIntent parseText(const std::string& text, 
                           IntentSource source,
                           const std::string& language) {
        auto start = std::chrono::high_resolution_clock::now();
        
        ParsedIntent intent;
        intent.timestamp_us = getTimestampUs();
        intent.raw_text = text;
        intent.language = language;
        intent.source = source;
        
        // Determine which model to use based on text content
        // First, quick keyword check for critical intents
        auto quick_intent = quickCriticalCheck(text);
        if (quick_intent.has_value()) {
            intent = quick_intent.value();
            intent.timestamp_us = getTimestampUs();
            intent.raw_text = text;
            intent.language = language;
            intent.source = source;
            
            auto end = std::chrono::high_resolution_clock::now();
            intent.processing_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
            
            updateStats(intent, intent.processing_time_ms);
            if (intent_callback_) intent_callback_(intent);
            return intent;
        }
        
        // Run appropriate model inference
        if (isCriticalIntent(text)) {
            intent = runCriticalModel(text, language);
            intent.source = source;
            intent.raw_text = text;
            intent.language = language;
        } else {
            intent = runMissionModel(text, language);
            intent.source = source;
            intent.raw_text = text;
            intent.language = language;
        }
        
        // Cloud fallback if confidence too low and enabled
        if (config_.enable_cloud_fallback && intent.confidence < config_.unknown_threshold && cloud_callback_) {
            if (auto cloud_intent = cloud_callback_(text, language)) {
                cloud_intent->on_device = false;
                intent = cloud_intent.value();
            }
            stats_.cloud_fallback_parses++;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        intent.processing_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
        intent.on_device = true;
        
        updateStats(intent, intent.processing_time_ms);
        
        if (intent_callback_) intent_callback_(intent);
        
        return intent;
    }
    
    ParsedIntent parseAudio(const std::string& audio_path,
                            IntentSource source,
                            const std::string& language) {
        // TODO: Integrate with ASR (whisper.cpp, etc.)
        // For now, return empty
        ParsedIntent intent;
        intent.timestamp_us = getTimestampUs();
        intent.raw_text = "";
        intent.language = language;
        intent.source = source;
        intent.type = IntentType::UNKNOWN;
        intent.confidence = 0.0f;
        return intent;
    }
    
    std::vector<ParsedIntent> parseBatch(const std::vector<std::string>& texts,
                                         IntentSource source,
                                         const std::string& language) {
        std::vector<ParsedIntent> results;
        results.reserve(texts.size());
        
        for (const auto& text : texts) {
            results.push_back(parseText(text, source, language));
        }
        
        return results;
    }
    
    ModelInfo getModelInfo() const {
        ModelInfo info;
        info.critical_model_name = critical_model_name_;
        info.mission_model_name = mission_model_name_;
        info.critical_model_size_mb = critical_model_size_mb_;
        info.mission_model_size_mb = mission_model_size_mb_;
        info.runtime = config_.runtime;
        info.critical_model_loaded = critical_model_loaded_;
        info.mission_model_loaded = mission_model_loaded_;
        return info;
    }
    
    const Stats& getStats() const {
        return stats_;
    }
    
    bool reloadModels(const IntentParserConfig& config) {
        config_ = config;
        critical_model_loaded_ = false;
        mission_model_loaded_ = false;
        return initialize(config);
    }
    
    void setIntentCallback(std::function<void(const ParsedIntent&)> cb) {
        intent_callback_ = cb;
    }
    
    void setCloudCallback(std::function<std::optional<ParsedIntent>(const std::string&, const std::string&)> cb) {
        cloud_callback_ = cb;
    }

private:
    IntentParserConfig config_;
    bool initialized_ = false;
    bool critical_model_loaded_ = false;
    bool mission_model_loaded_ = false;
    std::string critical_model_name_;
    std::string mission_model_name_;
    size_t critical_model_size_mb_ = 0;
    size_t mission_model_size_mb_ = 0;
    
    // Tokenizer
    std::map<std::string, int> vocab_;
    std::map<int, std::string> id_to_token_;
    
    // Model handles (TensorRT/ONNX Runtime)
    void* critical_engine_ = nullptr;
    void* mission_engine_ = nullptr;
    void* critical_context_ = nullptr;
    void* mission_context_ = nullptr;
    
    // Label mappings
    std::vector<std::string> critical_labels_;
    std::vector<std::string> mission_labels_;
    
    Stats stats_;
    std::mutex stats_mutex_;
    
    std::function<void(const ParsedIntent&)> intent_callback_;
    std::function<std::optional<ParsedIntent>(const std::string&, const std::string&)> cloud_callback_;
    
    // Quick keyword-based critical intent detection
    std::optional<ParsedIntent> quickCriticalCheck(const std::string& text) {
        static const std::map<std::string, std::pair<IntentType, float>> critical_keywords = {
            {"emergency stop", {IntentType::EMERGENCY_STOP, 0.99f}},
            {"stop immediately", {IntentType::EMERGENCY_STOP, 0.98f}},
            {"stop now", {IntentType::EMERGENCY_STOP, 0.97f}},
            {"return home", {IntentType::RETURN_HOME, 0.95f}},
            {"go home", {IntentType::RETURN_HOME, 0.93f}},
            {"come back", {IntentType::RETURN_HOME, 0.92f}},
            {"hold position", {IntentType::HOLD_POSITION, 0.95f}},
            {"stay there", {IntentType::HOLD_POSITION, 0.93f}},
            {"hover", {IntentType::HOLD_POSITION, 0.9f}},
            {"land now", {IntentType::LAND_NOW, 0.96f}},
            {"emergency land", {IntentType::LAND_NOW, 0.98f}},
            {"abort mission", {IntentType::ABORT_MISSION, 0.95f}},
            {"cancel mission", {IntentType::ABORT_MISSION, 0.93f}},
            {"mission abort", {IntentType::ABORT_MISSION, 0.94f}}
        };
        
        std::string lower_text = text;
        std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), ::tolower);
        
        for (const auto& [keyword, intent_conf] : critical_keywords) {
            if (lower_text.find(keyword) != std::string::npos) {
                ParsedIntent intent;
                intent.type = intent_conf.first;
                intent.confidence = intent_conf.second;
                intent.priority = IntentPriority::CRITICAL;
                return intent;
            }
        }
        return std::nullopt;
    }
    
    bool isCriticalIntent(const std::string& text) {
        return quickCriticalCheck(text).has_value();
    }
    
    ParsedIntent runCriticalModel(const std::string& text, const std::string& language) {
        ParsedIntent intent;
        intent.timestamp_us = getTimestampUs();
        
        if (!critical_model_loaded_) {
            // Fallback to keyword-based
            auto quick = quickCriticalCheck(text);
            if (quick.has_value()) {
                return quick.value();
            }
            intent.type = IntentType::UNKNOWN;
            intent.confidence = 0.0f;
            return intent;
        }
        
        // Tokenize
        auto input_ids = tokenize(text);
        if (input_ids.empty()) {
            intent.type = IntentType::UNKNOWN;
            intent.confidence = 0.0f;
            return intent;
        }
        
        // Run inference (placeholder - would use TensorRT/ONNX Runtime)
        auto inference_result = runInference(critical_engine_, critical_context_, input_ids);
        
        // Process output
        auto top_class = getTopClass(inference_result.logits, critical_labels_);
        
        intent.type = static_cast<IntentType>(top_class.first);
        intent.confidence = top_class.second;
        intent.priority = (top_class.second > config_.critical_threshold) ? IntentPriority::CRITICAL : IntentPriority::HIGH;
        
        // Extract parameters
        extractParameters(text, intent);
        
        return intent;
    }
    
    ParsedIntent runMissionModel(const std::string& text, const std::string& language) {
        ParsedIntent intent;
        intent.timestamp_us = getTimestampUs();
        
        if (!mission_model_loaded_) {
            // Fallback to simple keyword matching
            intent = keywordMissionMatch(text);
            return intent;
        }
        
        // Tokenize
        auto input_ids = tokenize(text);
        if (input_ids.empty()) {
            intent.type = IntentType::UNKNOWN;
            intent.confidence = 0.0f;
            return intent;
        }
        
        // Run inference (placeholder)
        auto inference_result = runInference(mission_engine_, mission_context_, input_ids);
        
        // Process output
        auto top_class = getTopClass(inference_result.logits, mission_labels_);
        
        intent.type = static_cast<IntentType>(top_class.first + 10);  // Mission types start at 10
        intent.confidence = top_class.second;
        intent.priority = (top_class.second > config_.mission_threshold) ? IntentPriority::HIGH : IntentPriority::NORMAL;
        
        // Extract parameters
        extractParameters(text, intent);
        
        return intent;
    }
    
    std::vector<int> tokenize(const std::string& text) {
        // Simplified tokenization - would use actual tokenizer (WordPiece/BPE)
        std::vector<int> tokens;
        // Add [CLS] token
        tokens.push_back(vocab_.count("[CLS]") ? vocab_["[CLS]"] : 101);
        
        // Simple whitespace tokenization for demo
        std::string word;
        for (char c : text) {
            if (std::isspace(c)) {
                if (!word.empty()) {
                    auto it = vocab_.find(word);
                    if (it != vocab_.end()) tokens.push_back(it->second);
                    else tokens.push_back(vocab_.count("[UNK]") ? vocab_["[UNK]"] : 100);
                    word.clear();
                }
            } else {
                word += std::tolower(c);
            }
        }
        if (!word.empty()) {
            auto it = vocab_.find(word);
            if (it != vocab_.end()) tokens.push_back(it->second);
            else tokens.push_back(vocab_.count("[UNK]") ? vocab_["[UNK]"] : 100);
        }
        
        // Add [SEP] token
        tokens.push_back(vocab_.count("[SEP]") ? vocab_["[SEP]"] : 102);
        
        // Pad or truncate
        if (tokens.size() > config_.max_seq_length) {
            tokens.resize(config_.max_seq_length);
        } else {
            while (tokens.size() < config_.max_seq_length) {
                tokens.push_back(vocab_.count("[PAD]") ? vocab_["[PAD]"] : 0);
            }
        }
        
        return tokens;
    }
    
    struct InferenceResult {
        std::vector<float> logits;
        float latency_ms = 0.0f;
    };
    
    InferenceResult runInference(void* engine, void* context, const std::vector<int>& input_ids) {
        // Placeholder for actual TensorRT/ONNX Runtime inference
        InferenceResult result;
        result.latency_ms = 5.0f;  // Simulated
        
        // Generate dummy logits for demo
        result.logits.resize(20, 0.0f);  // 20 classes
        return result;
    }
    
    std::pair<int, float> getTopClass(const std::vector<float>& logits, const std::vector<std::string>& labels) {
        if (logits.empty()) return {0, 0.0f};
        
        int max_idx = 0;
        float max_val = logits[0];
        
        for (size_t i = 1; i < logits.size(); ++i) {
            if (logits[i] > max_val) {
                max_val = logits[i];
                max_idx = i;
            }
        }
        
        // Softmax
        float sum_exp = 0.0f;
        for (float v : logits) sum_exp += std::exp(v - logits[max_idx]);
        float prob = std::exp(max_val - logits[max_idx]) / sum_exp;
        
        return {max_idx, prob};
    }
    
    void extractParameters(const std::string& text, ParsedIntent& intent) {
        // Extract parameters using regex/keyword matching
        // Waypoints, areas, targets, etc.
        
        // Example: "search area building 3" -> area: "building 3"
        // "inspect tower 5" -> target: "tower 5"
        // "fly to 100 200 50" -> waypoint: [100, 200, 50]
        
        // Simplified extraction
        std::string lower = text;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        
        // Area extraction
        if (lower.find("area") != std::string::npos) {
            size_t pos = lower.find("area");
            std::string after = lower.substr(pos + 4);
            // Extract next few words as area name
            intent.parameters["area"] = after.substr(0, 50);
        }
        
        // Target extraction
        if (lower.find("inspect") != std::string::npos || lower.find("target") != std::string::npos) {
            // Extract target description
            intent.parameters["target"] = "extracted_target";
        }
        
        // Waypoint extraction (coordinates)
        // Would use regex for numbers
    }
    
    ParsedIntent keywordMissionMatch(const std::string& text) {
        static const std::map<std::string, std::pair<IntentType, float>> mission_keywords = {
            {"search", {IntentType::SEARCH_AREA, 0.8f}},
            {"scan", {IntentType::SEARCH_AREA, 0.75f}},
            {"follow", {IntentType::FOLLOW_ME, 0.85f}},
            {"inspect", {IntentType::INSPECT_TARGET, 0.8f}},
            {"patrol", {IntentType::PATROL_ROUTE, 0.8f}},
            {"formation", {IntentType::CHANGE_FORMATION, 0.8f}},
            {"go to", {IntentType::GO_TO_WAYPOINT, 0.8f}},
            {"fly to", {IntentType::GO_TO_WAYPOINT, 0.75f}},
            {"takeoff", {IntentType::TAKEOFF, 0.9f}},
            {"land", {IntentType::LAND_AT, 0.85f}}
        };
        
        std::string lower = text;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        
        for (const auto& [keyword, intent_conf] : mission_keywords) {
            if (lower.find(keyword) != std::string::npos) {
                ParsedIntent intent;
                intent.type = intent_conf.first;
                intent.confidence = intent_conf.second;
                intent.priority = IntentPriority::HIGH;
                return intent;
            }
        }
        
        ParsedIntent intent;
        intent.type = IntentType::UNKNOWN;
        intent.confidence = 0.0f;
        return intent;
    }
    
    bool loadTokenizer(const std::string& path) {
        // Load vocab from file
        std::ifstream file(path);
        if (!file.is_open()) {
            // Use default vocab
            vocab_["[PAD]"] = 0;
            vocab_["[UNK]"] = 100;
            vocab_["[CLS]"] = 101;
            vocab_["[SEP]"] = 102;
            vocab_["[MASK]"] = 103;
            return true;
        }
        
        std::string line;
        int idx = 0;
        while (std::getline(file, line)) {
            if (!line.empty()) {
                vocab_[line] = idx++;
            }
        }
        return true;
    }
    
    bool loadCriticalModel(const std::string& path) {
        critical_model_name_ = path;
        // Load TensorRT engine or ONNX model
        // Placeholder
        critical_model_loaded_ = !path.empty();
        critical_model_size_mb_ = 25;  // ~25MB for DistilBERT-INT8
        return critical_model_loaded_;
    }
    
    bool loadMissionModel(const std::string& path) {
        mission_model_name_ = path;
        // Load TensorRT engine or ONNX model
        mission_model_loaded_ = !path.empty();
        mission_model_size_mb_ = 42;  // ~42MB for BERT-base-INT8
        return mission_model_loaded_;
    }
    
    void warmup(int iterations) {
        std::string test_text = "emergency stop return home search area";
        for (int i = 0; i < iterations; ++i) {
            runCriticalModel(test_text, "en");
            runMissionModel(test_text, "en");
        }
    }
    
    void updateStats(const ParsedIntent& intent, float latency_ms) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.total_parses++;
        if (intent.type == IntentType::EMERGENCY_STOP || 
            intent.type == IntentType::RETURN_HOME ||
            intent.type == IntentType::HOLD_POSITION ||
            intent.type == IntentType::LAND_NOW ||
            intent.type == IntentType::ABORT_MISSION) {
            stats_.critical_parses++;
            stats_.avg_critical_latency_ms = 0.9f * stats_.avg_critical_latency_ms + 0.1f * latency_ms;
        } else if (intent.type != IntentType::UNKNOWN) {
            stats_.mission_parses++;
            stats_.avg_mission_latency_ms = 0.9f * stats_.avg_mission_latency_ms + 0.1f * latency_ms;
        }
        
        stats_.avg_latency_ms = 0.9f * stats_.avg_latency_ms + 0.1f * latency_ms;
        stats_.avg_confidence = 0.9f * stats_.avg_confidence + 0.1f * intent.confidence;
    }
    
    uint64_t getTimestampUs() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
};

IntentParser::IntentParser() : pimpl_(std::make_unique<Impl>()) {}
IntentParser::~IntentParser() = default;
IntentParser::IntentParser(IntentParser&&) noexcept = default;
IntentParser& IntentParser::operator=(IntentParser&&) noexcept = default;

bool IntentParser::initialize(const IntentParserConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

ParsedIntent IntentParser::parseText(const std::string& text, 
                                     IntentSource source,
                                     const std::string& language) {
    return pimpl_->parseText(text, source, language);
}

ParsedIntent IntentParser::parseAudio(const std::string& audio_path,
                                      IntentSource source,
                                      const std::string& language) {
    return pimpl_->parseAudio(audio_path, source, language);
}

std::vector<ParsedIntent> IntentParser::parseBatch(const std::vector<std::string>& texts,
                                                   IntentSource source,
                                                   const std::string& language) {
    return pimpl_->parseBatch(texts, source, language);
}

IntentParser::ModelInfo IntentParser::getModelInfo() const {
    return pimpl_->getModelInfo();
}

const IntentParser::Stats& IntentParser::getStats() const {
    return pimpl_->getStats();
}

bool IntentParser::reloadModels(const IntentParserConfig& config) {
    return pimpl_->reloadModels(config);
}

bool IntentParser::isInitialized() const {
    return initialized_;
}

void IntentParser::setIntentCallback(IntentCallback cb) {
    intent_callback_ = cb;
    pimpl_->setIntentCallback(cb);
}

void IntentParser::setCloudCallback(CloudCallback cb) {
    cloud_callback_ = cb;
    pimpl_->setCloudCallback(cb);
}

std::string ParsedIntent::to_json() const {
    json j;
    j["type"] = static_cast<int>(type);
    j["source"] = static_cast<int>(source);
    j["priority"] = static_cast<int>(priority);
    j["confidence"] = confidence;
    j["raw_text"] = raw_text;
    j["language"] = language;
    j["parameters"] = parameters;
    j["timestamp_us"] = timestamp_us;
    j["processing_time_ms"] = processing_time_ms;
    j["on_device"] = on_device;
    return j.dump();
}

}  // namespace swarm::interface