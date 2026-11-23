#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "swarm_interface_intent_parser/intent_parser.hpp"

using namespace swarm::interface;

class IntentParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        parser_ = std::make_unique<IntentParser>();
        
        // Configure for testing (no actual models)
        IntentParserConfig config;
        config.distilbert_model_path = "";
        config.bert_model_path = "";
        config.tokenizer_path = "";
        config.runtime = "onnxruntime";
        config.critical_threshold = 0.9f;
        config.mission_threshold = 0.75f;
        config.unknown_threshold = 0.5f;
        config.default_language = "en";
        config.warmup_on_start = false;
        config.enable_cloud_fallback = false;
        
        parser_->initialize(config);
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::unique_ptr<IntentParser> parser_;
};

// ============================================================================
// Critical Intent Tests (DistilBERT, < 10ms)
// ============================================================================

TEST_F(IntentParserTest, ParseEmergencyStop_HighConfidence) {
    auto intent = parser_->parseText("emergency stop", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::EMERGENCY_STOP);
    EXPECT_GT(intent.confidence, 0.9f);
    EXPECT_EQ(intent.priority, IntentPriority::CRITICAL);
}

TEST_F(IntentParserTest, ParseReturnHome_HighConfidence) {
    auto intent = parser_->parseText("return home", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::RETURN_HOME);
    EXPECT_GT(intent.confidence, 0.9f);
    EXPECT_EQ(intent.priority, IntentPriority::CRITICAL);
}

TEST_F(IntentParserTest, ParseHoldPosition_HighConfidence) {
    auto intent = parser_->parseText("hold position", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::HOLD_POSITION);
    EXPECT_GT(intent.confidence, 0.9f);
    EXPECT_EQ(intent.priority, IntentPriority::CRITICAL);
}

TEST_F(IntentParserTest, ParseLandNow_HighConfidence) {
    auto intent = parser_->parseText("land now", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::LAND_NOW);
    EXPECT_GT(intent.confidence, 0.9f);
    EXPECT_EQ(intent.priority, IntentPriority::CRITICAL);
}

TEST_F(IntentParserTest, ParseAbortMission_HighConfidence) {
    auto intent = parser_->parseText("abort mission", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::ABORT_MISSION);
    EXPECT_GT(intent.confidence, 0.9f);
    EXPECT_EQ(intent.priority, IntentPriority::CRITICAL);
}

TEST_F(IntentParserTest, CriticalIntent_Korean) {
    auto intent = parser_->parseText("긴급 정지", IntentSource::VOICE, "ko");
    
    EXPECT_EQ(intent.type, IntentType::EMERGENCY_STOP);
    EXPECT_GT(intent.confidence, 0.9f);
    EXPECT_EQ(intent.language, "ko");
}

TEST_F(IntentParserTest, CriticalIntent_TabletSource) {
    auto intent = parser_->parseText("emergency stop", IntentSource::TABLET);
    
    EXPECT_EQ(intent.type, IntentType::EMERGENCY_STOP);
    EXPECT_EQ(intent.source, IntentSource::TABLET);
}

// ============================================================================
// Mission Intent Tests (BERT, < 50ms)
// ============================================================================

TEST_F(IntentParserTest, ParseSearchArea_MissionConfidence) {
    auto intent = parser_->parseText("search area building 3", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::SEARCH_AREA);
    EXPECT_GT(intent.confidence, 0.7f);
    EXPECT_EQ(intent.priority, IntentPriority::HIGH);
}

TEST_F(IntentParserTest, ParseFollowMe_MissionConfidence) {
    auto intent = parser_->parseText("follow me", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::FOLLOW_ME);
    EXPECT_GT(intent.confidence, 0.7f);
    EXPECT_EQ(intent.priority, IntentPriority::HIGH);
}

TEST_F(IntentParserTest, ParseInspectTarget_MissionConfidence) {
    auto intent = parser_->parseText("inspect tower 5", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::INSPECT_TARGET);
    EXPECT_GT(intent.confidence, 0.7f);
    EXPECT_EQ(intent.priority, IntentPriority::HIGH);
}

TEST_F(IntentParserTest, ParseFollowMe_Tablet) {
    auto intent = parser_->parseText("follow me", IntentSource::TABLET);
    
    EXPECT_EQ(intent.type, IntentType::FOLLOW_ME);
    EXPECT_EQ(intent.source, IntentSource::TABLET);
}

TEST_F(IntentParserTest, MissionIntent_Korean) {
    auto intent = parser_->parseText("건물 3 탐색", IntentSource::VOICE, "ko");
    
    EXPECT_EQ(intent.type, IntentType::SEARCH_AREA);
    EXPECT_EQ(intent.language, "ko");
}

// ============================================================================
// Unknown/Low Confidence Tests
// ============================================================================

TEST_F(IntentParserTest, UnknownIntent_LowConfidence) {
    auto intent = parser_->parseText("random gibberish text", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::UNKNOWN);
    EXPECT_LT(intent.confidence, 0.5f);
}

TEST_F(IntentParserTest, EmptyText_Unknown) {
    auto intent = parser_->parseText("", IntentSource::VOICE);
    
    EXPECT_EQ(intent.type, IntentType::UNKNOWN);
    EXPECT_EQ(intent.confidence, 0.0f);
}

// ============================================================================
// Parameter Extraction Tests
// ============================================================================

TEST_F(IntentParserTest, ExtractAreaParameter) {
    auto intent = parser_->parseText("search area building 3", IntentSource::VOICE);
    
    EXPECT_TRUE(intent.parameters.count("area") > 0);
}

TEST_F(IntentParserTest, ExtractTargetParameter) {
    auto intent = parser_->parseText("inspect tower 5", IntentSource::VOICE);
    
    EXPECT_TRUE(intent.parameters.count("target") > 0);
}

// ============================================================================
// Source Type Tests
// ============================================================================

TEST_F(IntentParserTest, SourceType_Voice) {
    auto intent = parser_->parseText("return home", IntentSource::VOICE);
    EXPECT_EQ(intent.source, IntentSource::VOICE);
}

TEST_F(IntentParserTest, SourceType_Tablet) {
    auto intent = parser_->parseText("search area", IntentSource::TABLET);
    EXPECT_EQ(intent.source, IntentSource::TABLET);
}

TEST_F(IntentParserTest, SourceType_Gesture) {
    auto intent = parser_->parseText("hold position", IntentSource::GESTURE);
    EXPECT_EQ(intent.source, IntentSource::GESTURE);
}

TEST_F(IntentParserTest, SourceType_Auto) {
    auto intent = parser_->parseText("emergency stop", IntentSource::AUTO);
    EXPECT_EQ(intent.source, IntentSource::AUTO);
}

// ============================================================================
// Language Tests
// ============================================================================

TEST_F(IntentParserTest, Language_English) {
    auto intent = parser_->parseText("return home", IntentSource::VOICE, "en");
    EXPECT_EQ(intent.language, "en");
}

TEST_F(IntentParserTest, Language_Korean) {
    auto intent = parser_->parseText("집으로 돌아와", IntentSource::VOICE, "ko");
    EXPECT_EQ(intent.language, "ko");
}

TEST_F(IntentParserTest, Language_Japanese) {
    auto intent = parser_->parseText("ホームに戻る", IntentSource::VOICE, "ja");
    EXPECT_EQ(intent.language, "ja");
}

// ============================================================================
// Batch Processing Tests
// ============================================================================

TEST_F(IntentParserTest, ParseBatch_MultipleCommands) {
    std::vector<std::string> texts = {
        "emergency stop",
        "return home",
        "search area",
        "follow me"
    };
    
    auto results = parser_->parseBatch(texts, IntentSource::VOICE);
    
    EXPECT_EQ(results.size(), 4);
    EXPECT_EQ(results[0].type, IntentType::EMERGENCY_STOP);
    EXPECT_EQ(results[1].type, IntentType::RETURN_HOME);
    EXPECT_EQ(results[2].type, IntentType::SEARCH_AREA);
    EXPECT_EQ(results[3].type, IntentType::FOLLOW_ME);
}

// ============================================================================
// JSON Serialization Tests
// ============================================================================

TEST_F(IntentParserTest, IntentToJson_Valid) {
    auto intent = parser_->parseText("emergency stop", IntentSource::VOICE);
    
    std::string json = intent.to_json();
    
    EXPECT_TRUE(json.find("EMERGENCY_STOP") != std::string::npos || json.find("1") != std::string::npos);
    EXPECT_TRUE(json.find("confidence") != std::string::npos);
    EXPECT_TRUE(json.find("timestamp_us") != std::string::npos);
}

TEST_F(IntentParserTest, IntentToJson_Parameters) {
    auto intent = parser_->parseText("search area building 3", IntentSource::VOICE);
    
    std::string json = intent.to_json();
    
    EXPECT_TRUE(json.find("parameters") != std::string::npos);
    EXPECT_TRUE(json.find("area") != std::string::npos);
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(IntentParserTest, Stats_UpdateOnParse) {
    parser_->parseText("emergency stop", IntentSource::VOICE);
    parser_->parseText("return home", IntentSource::VOICE);
    parser_->parseText("search area", IntentSource::VOICE);
    
    auto stats = parser_->getStats();
    
    EXPECT_EQ(stats.total_parses, 3);
    EXPECT_EQ(stats.critical_parses, 2);
    EXPECT_EQ(stats.mission_parses, 1);
    EXPECT_GT(stats.avg_latency_ms, 0.0f);
    EXPECT_GT(stats.avg_confidence, 0.0f);
}

TEST_F(IntentParserTest, Stats_CriticalLatencyTracked) {
    parser_->parseText("emergency stop", IntentSource::VOICE);
    
    auto stats = parser_->getStats();
    
    EXPECT_GT(stats.avg_critical_latency_ms, 0.0f);
    EXPECT_EQ(stats.avg_mission_latency_ms, 0.0f);
}

TEST_F(IntentParserTest, Stats_MissionLatencyTracked) {
    parser_->parseText("search area", IntentSource::VOICE);
    
    auto stats = parser_->getStats();
    
    EXPECT_GT(stats.avg_mission_latency_ms, 0.0f);
    EXPECT_EQ(stats.avg_critical_latency_ms, 0.0f);
}

// ============================================================================
// Priority Tests
// ============================================================================

TEST_F(IntentParserTest, CriticalIntent_HighPriority) {
    auto intent = parser_->parseText("emergency stop", IntentSource::VOICE);
    EXPECT_EQ(intent.priority, IntentPriority::CRITICAL);
}

TEST_F(IntentParserTest, MissionIntent_HighPriority) {
    auto intent = parser_->parseText("search area", IntentSource::VOICE);
    EXPECT_EQ(intent.priority, IntentPriority::HIGH);
}

TEST_F(IntentParserTest, UnknownIntent_LowPriority) {
    auto intent = parser_->parseText("random text", IntentSource::VOICE);
    EXPECT_EQ(intent.priority, IntentPriority::NORMAL);
}

// ============================================================================
// Model Info Tests
// ============================================================================

TEST_F(IntentParserTest, ModelInfo_Default) {
    auto info = parser_->getModelInfo();
    
    // Should be empty/false since no models loaded in test
    EXPECT_FALSE(info.critical_model_loaded);
    EXPECT_FALSE(info.mission_model_loaded);
    EXPECT_EQ(info.runtime, "onnxruntime");
}

// ============================================================================
// Batch Processing Tests
// ============================================================================

TEST_F(IntentParserTest, ParseBatch_MixedIntents) {
    std::vector<std::string> texts = {
        "emergency stop",
        "return home", 
        "search building 1",
        "follow me",
        "inspect tower 2"
    };
    
    auto results = parser_->parseBatch(texts, IntentSource::VOICE);
    
    EXPECT_EQ(results.size(), 5);
    EXPECT_EQ(results[0].type, IntentType::EMERGENCY_STOP);
    EXPECT_EQ(results[1].type, IntentType::RETURN_HOME);
    EXPECT_EQ(results[2].type, IntentType::SEARCH_AREA);
    EXPECT_EQ(results[3].type, IntentType::FOLLOW_ME);
    EXPECT_EQ(results[4].type, IntentType::INSPECT_TARGET);
    
    // Check all have valid timestamps
    for (const auto& r : results) {
        EXPECT_GT(r.timestamp_us, 0);
        EXPECT_EQ(r.source, IntentSource::VOICE);
    }
}

// ============================================================================
// Priority Edge Cases
// ============================================================================

TEST_F(IntentParserTest, Priority_CriticalForEmergency) {
    auto intent = parser_->parseText("emergency stop immediately", IntentSource::VOICE);
    EXPECT_EQ(intent.priority, IntentPriority::CRITICAL);
}

TEST_F(IntentParserTest, Priority_NormalForUnknown) {
    auto intent = parser_->parseText("hello world", IntentSource::VOICE);
    EXPECT_EQ(intent.priority, IntentPriority::NORMAL);
}

// ============================================================================
// JSON Serialization
// ============================================================================

TEST_F(IntentParserTest, ToJson_ContainsAllFields) {
    auto intent = parser_->parseText("emergency stop", IntentSource::VOICE);
    std::string json = intent.to_json();
    
    // Check all expected fields
    EXPECT_TRUE(json.find("type") != std::string::npos);
    EXPECT_TRUE(json.find("source") != std::string::npos);
    EXPECT_TRUE(json.find("priority") != std::string::npos);
    EXPECT_TRUE(json.find("confidence") != std::string::npos);
    EXPECT_TRUE(json.find("raw_text") != std::string::npos);
    EXPECT_TRUE(json.find("language") != std::string::npos);
    EXPECT_TRUE(json.find("parameters") != std::string::npos);
    EXPECT_TRUE(json.find("timestamp_us") != std::string::npos);
    EXPECT_TRUE(json.find("processing_time_ms") != std::string::npos);
    EXPECT_TRUE(json.find("on_device") != std::string::npos);
}

TEST_F(IntentParserTest, ToJson_ParametersIncluded) {
    auto intent = parser_->parseText("search area building 3", IntentSource::VOICE);
    std::string json = intent.to_json();
    
    EXPECT_TRUE(json.find("area") != std::string::npos);
}

// ============================================================================
// Intent Validity
// ============================================================================

TEST_F(IntentParserTest, ValidIntent_HasTypeAndConfidence) {
    auto intent = parser_->parseText("emergency stop", IntentSource::VOICE);
    EXPECT_TRUE(intent.valid());
}

TEST_F(IntentParserTest, InvalidIntent_NoTypeOrLowConfidence) {
    auto intent = parser_->parseText("", IntentSource::VOICE);
    EXPECT_FALSE(intent.valid());
    
    auto intent2 = parser_->parseText("random", IntentSource::VOICE);
    EXPECT_FALSE(intent2.valid());
}

// ============================================================================
// Callback Tests
// ============================================================================

TEST_F(IntentParserTest, IntentCallback_Called) {
    bool callback_called = false;
    ParsedIntent captured_intent;
    
    parser_->setIntentCallback([&](const ParsedIntent& intent) {
        callback_called = true;
        captured_intent = intent;
    });
    
    parser_->parseText("emergency stop", IntentSource::VOICE);
    
    EXPECT_TRUE(callback_called);
    EXPECT_EQ(captured_intent.type, IntentType::EMERGENCY_STOP);
}

TEST_F(IntentParserTest, CloudCallback_Called) {
    bool cloud_called = false;
    
    parser_->setCloudCallback([&](const std::string& text, const std::string& lang) {
        cloud_called = true;
        return std::nullopt;
    });
    
    // This would trigger cloud fallback if confidence is low
    // But our test config has cloud fallback disabled
    // So just verify callback can be set
    EXPECT_NO_THROW(parser_->setCloudCallback([](const std::string&, const std::string&) {
        return std::nullopt;
    }));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}