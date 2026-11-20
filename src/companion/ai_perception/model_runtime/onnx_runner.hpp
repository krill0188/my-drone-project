#ifndef ONNX_RUNNER_HPP
#define ONNX_RUNNER_HPP

#include "ai_inference.hpp"
#include <string>
#include <vector>
#include <memory>

namespace swarm::ai {

struct ONNXRunnerConfig {
    std::string model_path;
    std::string execution_provider = "cuda";  // "cuda", "cpu", "tensorrt", "dml"
    int device_id = 0;
    int intra_op_threads = 4;
    int inter_op_threads = 4;
    bool enable_mem_pattern = true;
    bool enable_cpu_mem_arena = true;
    std::vector<int64_t> input_shape;  // Override input shape
};

struct ONNXRunner {
    ONNXRunner() = default;
    ~ONNXRunner();
    
    ONNXRunner(const ONNXRunner&) = delete;
    ONNXRunner& operator=(const ONNXRunner&) = delete;
    ONNXRunner(ONNXRunner&&) noexcept;
    ONNXRunner& operator=(ONNXRunner&&) noexcept;
    
    // Load model
    bool loadModel(const ONNXRunnerConfig& config);
    
    // Run inference
    bool infer(const float* input_data, size_t input_size,
               std::vector<float*>& outputs, std::vector<size_t>& output_sizes,
               float& latency_ms);
    
    // Run inference with named inputs/outputs
    bool infer(const std::vector<std::pair<std::string, const float*>>& inputs,
               std::vector<std::pair<std::string, float*>>& outputs,
               float& latency_ms);
    
    // Get model metadata
    ModelInfo getModelInfo() const;
    
    // Get input/output info
    struct TensorInfo {
        std::string name;
        std::vector<int64_t> shape;
        size_t element_count;
        std::string data_type;  // "float32", "float16", "int8", etc.
    };
    
    std::vector<TensorInfo> getInputInfos() const;
    std::vector<TensorInfo> getOutputInfos() const;
    
    // Check if loaded
    bool isLoaded() const { return session_ != nullptr; }
    
    // Warmup
    void warmup(int iterations = 10);
    
    // Get execution provider name
    std::string getExecutionProvider() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    void* session_ = nullptr;
    ModelInfo model_info_;
    std::vector<TensorInfo> input_infos_;
    std::vector<TensorInfo> output_infos_;
    std::string execution_provider_;
};

}  // namespace swarm::ai

#endif  // ONNX_RUNNER_HPP