#ifndef INFERENCE_ENGINE_HPP
#define INFERENCE_ENGINE_HPP

#include "ai_inference.hpp"
#include "tensorrt_builder.hpp"
#include "onnx_runner.hpp"
#include <memory>
#include <functional>
#include <mutex>

namespace swarm::ai {

struct InferenceEngineConfig {
    // Model paths
    std::string obstacle_model_path;
    std::string landing_model_path;
    std::string fault_model_path;
    std::string anomaly_model_path;
    
    // Runtime preferences
    std::string preferred_runtime = "tensorrt";  // "tensorrt", "onnxruntime"
    std::string fallback_runtime = "onnxruntime";
    std::string preferred_precision = "fp16";    // "fp32", "fp16", "int8"
    
    // TensorRT settings
    int max_workspace_size_mb = 1024;
    bool enable_fp16 = true;
    bool enable_int8 = false;
    std::string calibration_cache_dir;
    std::string calibration_data_dir;
    
    // ONNX Runtime settings
    int ort_device_id = 0;
    int ort_intra_op_threads = 4;
    int ort_inter_op_threads = 4;
    
    // Inference settings
    float confidence_threshold = 0.5f;
    float nms_threshold = 0.45f;
    int max_detections = 100;
    int max_batch_size = 1;
    
    // Device
    int gpu_device_id = 0;
    int dla_core = -1;
    
    // Preprocessing
    std::vector<float> input_mean = {0.485f, 0.456f, 0.406f};
    std::vector<float> input_std = {0.229f, 0.224f, 0.225f};
    int input_height = 640;
    int input_width = 640;
    
    // Postprocessing
    bool apply_nms = true;
    bool letterbox = true;
    
    // Performance
    bool enable_profiling = true;
    int warmup_iterations = 10;
};

struct PreprocessResult {
    std::vector<float> input_tensor;  // Flattened NCHW
    std::vector<int64_t> input_shape; // {1, 3, H, W}
    Eigen::Matrix3f transform_matrix; // For coordinate mapping back
    float scale = 1.0f;
    int pad_top = 0, pad_left = 0;
};

struct Detection {
    int class_id;
    std::string class_name;
    float confidence;
    Eigen::Vector4f bbox_xyxy;  // x1, y1, x2, y2 in original image coords
    Eigen::Vector3f position_enu;  // 3D position if available
    Eigen::Vector3f size;          // 3D size if available
    Eigen::Quaternionf orientation; // 3D orientation if available
};

struct InferenceResult {
    std::vector<Detection> detections;
    float preprocess_ms = 0.0f;
    float inference_ms = 0.0f;
    float postprocess_ms = 0.0f;
    float total_ms = 0.0f;
    bool used_fallback = false;
    std::string runtime_used;
    std::string model_name;
};

class InferenceEngine {
public:
    InferenceEngine();
    ~InferenceEngine();
    
    InferenceEngine(const InferenceEngine&) = delete;
    InferenceEngine& operator=(const InferenceEngine&) = delete;
    InferenceEngine(InferenceEngine&&) noexcept;
    InferenceEngine& operator=(InferenceEngine&&) noexcept;
    
    // Initialize engine
    bool initialize(const InferenceEngineConfig& config);
    
    // Run inference on image (obstacle detection)
    InferenceResult inferObstacle(const cv::Mat& image, uint16_t drone_id, uint64_t timestamp_us);
    
    // Run inference on image (landing site assessment)
    InferenceResult inferLanding(const cv::Mat& image, uint16_t drone_id, uint64_t timestamp_us);
    
    // Run inference on sensor data (fault detection)
    InferenceResult inferFault(const std::vector<float>& sensor_sequence, 
                               uint16_t drone_id, uint64_t timestamp_us);
    
    // Generic inference
    InferenceResult infer(const std::vector<float>& input_tensor,
                          const std::vector<int64_t>& input_shape,
                          InferenceType type,
                          uint16_t drone_id,
                          uint64_t timestamp_us);
    
    // Preprocess image for detection
    PreprocessResult preprocessImage(const cv::Mat& image);
    
    // Postprocess obstacle detection output
    std::vector<Detection> postprocessObstacle(const float* output, 
                                               const std::vector<int64_t>& output_shape,
                                               const PreprocessResult& preprocess_info);
    
    // Postprocess landing classification output
    std::vector<Detection> postprocessLanding(const float* output,
                                              const std::vector<int64_t>& output_shape);
    
    // Postprocess fault detection output
    std::vector<Detection> postprocessFault(const float* output,
                                            const std::vector<int64_t>& output_shape);
    
    // Convert detections to AIInferenceResult
    std::vector<AIInferenceResult> detectionsToInferenceResults(
        const std::vector<Detection>& detections,
        InferenceType type,
        uint16_t drone_id,
        uint64_t timestamp_us,
        float inference_ms,
        const std::string& model_name,
        const std::string& runtime);
    
    // Get model info
    ModelInfo getObstacleModelInfo() const;
    ModelInfo getLandingModelInfo() const;
    ModelInfo getFaultModelInfo() const;
    ModelInfo getAnomalyModelInfo() const;
    
    // Get inference stats
    InferenceStats getStats(InferenceType type) const;
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Get current runtime
    std::string getCurrentRuntime(InferenceType type) const;
    
    // Switch runtime (for testing fallback)
    bool switchRuntime(InferenceType type, const std::string& runtime);
    
    // Warmup all models
    void warmup();
    
    // Callbacks
    using InferenceCallback = std::function<void(const AIInferenceResult&)>;
    void setInferenceCallback(InferenceCallback cb) { inference_callback_ = cb; }
    
    using ErrorCallback = std::function<void(InferenceType, const std::string&)>;
    void setErrorCallback(ErrorCallback cb) { error_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    bool initialized_ = false;
    InferenceEngineConfig config_;
    
    std::function<void(const AIInferenceResult&)> inference_callback_;
    std::function<void(InferenceType, const std::string&)> error_callback_;
};

}  // namespace swarm::ai

#endif  // INFERENCE_ENGINE_HPP