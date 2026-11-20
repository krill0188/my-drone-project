#ifndef TENSORRT_BUILDER_HPP
#define TENSORRT_BUILDER_HPP

#include "ai_inference.hpp"
#include <string>
#include <vector>
#include <memory>
#include <functional>

namespace swarm::ai {

// Forward declaration for TensorRT types
class ICudaEngine;
class IBuilder;
class IBuilderConfig;
class INetworkDefinition;
class IParser;
class IRuntime;
class IExecutionContext;
class IHostMemory;

struct TensorRTBuilderConfig {
    std::string onnx_model_path;
    std::string engine_output_path;
    std::string precision = "fp16";  // "fp32", "fp16", "int8"
    int max_batch_size = 1;
    size_t max_workspace_size_mb = 1024;
    bool enable_fp16 = true;
    bool enable_int8 = false;
    std::string calibration_cache_path;
    std::string calibration_data_path;  // Directory with calibration images
    int calibration_batch_size = 1;
    int num_calibration_batches = 100;
    std::vector<int64_t> input_shape;   // e.g., {1, 3, 640, 640}
    std::string input_name = "images";
    std::vector<std::string> output_names;
    bool strict_type_constraints = false;
    int dla_core = -1;  // -1 = GPU, 0/1 = DLA core
    bool allow_gpu_fallback = true;
    bool verbose = false;
};

struct BuildResult {
    bool success = false;
    std::string error_message;
    std::string engine_path;
    size_t engine_size_bytes = 0;
    float build_time_seconds = 0.0f;
    std::string precision_used;
    int max_batch_size = 1;
    std::vector<int64_t> input_shape;
    std::vector<std::string> output_names;
    std::vector<std::vector<int64_t>> output_shapes;
};

class IInt8Calibrator {
public:
    virtual ~IInt8Calibrator() = default;
    virtual int getBatchSize() const = 0;
    virtual bool getBatch(void* bindings[], const char* names[], int nbBindings) = 0;
    virtual const void* readCalibrationCache(size_t& length) = 0;
    virtual void writeCalibrationCache(const void* ptr, size_t length) = 0;
};

class TensorRTBuilder {
public:
    TensorRTBuilder();
    ~TensorRTBuilder();
    
    // Non-copyable, movable
    TensorRTBuilder(const TensorRTBuilder&) = delete;
    TensorRTBuilder& operator=(const TensorRTBuilder&) = delete;
    TensorRTBuilder(TensorRTBuilder&&) noexcept;
    TensorRTBuilder& operator=(TensorRTBuilder&&) noexcept;
    
    // Build engine from ONNX
    BuildResult buildEngine(const TensorRTBuilderConfig& config);
    
    // Build with INT8 calibration
    BuildResult buildEngineInt8(const TensorRTBuilderConfig& config, 
                                 std::unique_ptr<IInt8Calibrator> calibrator);
    
    // Check if engine exists and is valid
    static bool validateEngine(const std::string& engine_path, 
                               const std::vector<int64_t>& expected_input_shape = {});
    
    // Get engine info without building
    static ModelInfo getEngineInfo(const std::string& engine_path);
    
    // Convert precision string to enum
    static bool parsePrecision(const std::string& precision_str, 
                               bool& out_fp16, bool& out_int8);
    
    // Utility: get TensorRT version
    static std::string getTensorRTVersion();
    
    // Utility: check GPU compatibility
    static bool checkGPUCompatibility(int gpu_device_id = 0);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

// Calibrator implementation for image directory
class ImageDirectoryCalibrator : public IInt8Calibrator {
public:
    ImageDirectoryCalibrator(const std::string& data_dir,
                             const std::string& cache_file,
                             int batch_size,
                             int num_batches,
                             const std::vector<int64_t>& input_shape,
                             const std::string& input_name);
    
    ~ImageDirectoryCalibrator() override;
    
    int getBatchSize() const override;
    bool getBatch(void* bindings[], const char* names[], int nbBindings) override;
    const void* readCalibrationCache(size_t& length) override;
    void writeCalibrationCache(const void* ptr, size_t length) override;

private:
    std::string data_dir_;
    std::string cache_file_;
    int batch_size_;
    int num_batches_;
    std::vector<int64_t> input_shape_;
    std::string input_name_;
    std::vector<std::string> image_files_;
    size_t current_batch_ = 0;
    std::vector<float> batch_buffer_;
    std::vector<char> calibration_cache_;
    
    bool loadNextBatch();
    void preprocessImage(const std::string& image_path, float* output);
};

// Engine runner for inference
class TensorRTEngine {
public:
    TensorRTEngine();
    ~TensorRTEngine();
    
    TensorRTEngine(const TensorRTEngine&) = delete;
    TensorRTEngine& operator=(const TensorRTEngine&) = delete;
    TensorRTEngine(TensorRTEngine&&) noexcept;
    TensorRTEngine& operator=(TensorRTEngine&&) noexcept;
    
    // Load engine from file
    bool loadEngine(const std::string& engine_path);
    
    // Load engine from memory
    bool loadEngine(const void* engine_data, size_t engine_size);
    
    // Create execution context
    bool createContext();
    
    // Run inference (synchronous)
    bool infer(const void* input_data, size_t input_size,
               void* output_data, size_t output_size,
               float& latency_ms);
    
    // Run inference with multiple outputs
    bool infer(const std::vector<const void*>& inputs,
               std::vector<void*>& outputs,
               float& latency_ms);
    
    // Async inference with CUDA streams
    bool inferAsync(const std::vector<const void*>& inputs,
                    std::vector<void*>& outputs,
                    cudaStream_t stream,
                    float& latency_ms);
    
    // Get engine info
    ModelInfo getModelInfo() const;
    
    // Get binding info
    struct BindingInfo {
        std::string name;
        bool is_input;
        std::vector<int64_t> shape;
        size_t size_bytes;
        int data_type;  // TensorRT DataType enum
    };
    
    std::vector<BindingInfo> getBindingInfos() const;
    
    // Check if loaded
    bool isLoaded() const { return engine_ != nullptr; }
    
    // Get max batch size
    int getMaxBatchSize() const;
    
    // Get device memory pointers for zero-copy
    void* getDeviceInputBuffer(size_t binding_index);
    void* getDeviceOutputBuffer(size_t binding_index);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    void* engine_ = nullptr;
    void* context_ = nullptr;
    ModelInfo model_info_;
    std::vector<void*> device_buffers_;
    std::vector<BindingInfo> binding_infos_;
};

}  // namespace swarm::ai

#endif  // TENSORRT_BUILDER_HPP