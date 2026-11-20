#include "onnx_runner.hpp"
#include <chrono>
#include <algorithm>

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace swarm::ai {

#ifdef HAVE_ONNXRUNTIME

class ONNXRunner::Impl {
public:
    Impl() : env_(ORT_LOGGING_LEVEL_WARNING, "swarm_ai") {}
    ~Impl() = default;
    
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;
    std::string execution_provider_;
};

ONNXRunner::ONNXRunner() : pimpl_(std::make_unique<Impl>()) {}
ONNXRunner::~ONNXRunner() = default;
ONNXRunner::ONNXRunner(ONNXRunner&&) noexcept = default;
ONNXRunner& ONNXRunner::operator=(ONNXRunner&&) noexcept = default;

bool ONNXRunner::loadModel(const ONNXRunnerConfig& config) {
    try {
        pimpl_->session_options_.SetIntraOpNumThreads(config.intra_op_threads);
        pimpl_->session_options_.SetInterOpNumThreads(config.inter_op_threads);
        pimpl_->session_options_.EnableMemPattern(config.enable_mem_pattern);
        pimpl_->session_options_.EnableCpuMemArena(config.enable_cpu_mem_arena);
        
        // Set execution provider
        if (config.execution_provider == "cuda") {
            OrtCUDAProviderOptions cuda_options;
            cuda_options.device_id = config.device_id;
            cuda_options.arena_extend_strategy = 0;
            cuda_options.gpu_mem_limit = SIZE_MAX;
            cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
            cuda_options.do_copy_in_default_stream = 1;
            pimpl_->session_options_.AppendExecutionProvider_CUDA(cuda_options);
            pimpl_->execution_provider_ = "CUDAExecutionProvider";
        } else if (config.execution_provider == "tensorrt") {
            OrtTensorRTProviderOptions trt_options;
            trt_options.device_id = config.device_id;
            trt_options.trt_max_workspace_size = 1073741824;  // 1GB
            trt_options.trt_fp16_enable = 1;
            trt_options.trt_int8_enable = 0;
            pimpl_->session_options_.AppendExecutionProvider_TensorRT(trt_options);
            pimpl_->execution_provider_ = "TensorRTExecutionProvider";
        } else if (config.execution_provider == "dml") {
            OrtDmlProviderOptions dml_options;
            dml_options.device_id = config.device_id;
            pimpl_->session_options_.AppendExecutionProvider_DML(dml_options);
            pimpl_->execution_provider_ = "DMLExecutionProvider";
        } else {
            pimpl_->execution_provider_ = "CPUExecutionProvider";
        }
        
        // Create session
        pimpl_->session_ = std::make_unique<Ort::Session>(
            pimpl_->env_, config.model_path.c_str(), pimpl_->session_options_);
        
        session_ = pimpl_->session_.get();
        
        // Get input/output info
        size_t num_inputs = session_->GetInputCount();
        size_t num_outputs = session_->GetOutputCount();
        
        input_infos_.clear();
        output_infos_.clear();
        pimpl_->input_names_.clear();
        pimpl_->output_names_.clear();
        pimpl_->input_shapes_.clear();
        pimpl_->output_shapes_.clear();
        
        for (size_t i = 0; i < num_inputs; ++i) {
            auto name = session_->GetInputNameAllocated(i, pimpl_->allocator_);
            pimpl_->input_names_.push_back(name.get());
            
            auto type_info = session_->GetInputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            auto shape = tensor_info.GetShape();
            
            // Override shape if provided
            if (!config.input_shape.empty() && i == 0) {
                shape = config.input_shape;
            }
            
            pimpl_->input_shapes_.push_back(shape);
            
            TensorInfo info;
            info.name = name.get();
            info.shape = shape;
            info.element_count = 1;
            for (auto dim : shape) info.element_count *= std::max<int64_t>(dim, 1);
            
            ONNXTensorElementDataType type = tensor_info.GetElementType();
            switch (type) {
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: info.data_type = "float32"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16: info.data_type = "float16"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8: info.data_type = "int8"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8: info.data_type = "uint8"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16: info.data_type = "int16"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16: info.data_type = "uint16"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32: info.data_type = "int32"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32: info.data_type = "uint32"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64: info.data_type = "int64"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64: info.data_type = "uint64"; break;
                default: info.data_type = "unknown"; break;
            }
            
            input_infos_.push_back(info);
        }
        
        for (size_t i = 0; i < num_outputs; ++i) {
            auto name = session_->GetOutputNameAllocated(i, pimpl_->allocator_);
            pimpl_->output_names_.push_back(name.get());
            
            auto type_info = session_->GetOutputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            auto shape = tensor_info.GetShape();
            
            pimpl_->output_shapes_.push_back(shape);
            
            TensorInfo info;
            info.name = name.get();
            info.shape = shape;
            info.element_count = 1;
            for (auto dim : shape) info.element_count *= std::max<int64_t>(dim, 1);
            
            ONNXTensorElementDataType type = tensor_info.GetElementType();
            switch (type) {
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: info.data_type = "float32"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16: info.data_type = "float16"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8: info.data_type = "int8"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8: info.data_type = "uint8"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16: info.data_type = "int16"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16: info.data_type = "uint16"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32: info.data_type = "int32"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32: info.data_type = "uint32"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64: info.data_type = "int64"; break;
                case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64: info.data_type = "uint64"; break;
                default: info.data_type = "unknown"; break;
            }
            
            output_infos_.push_back(info);
        }
        
        // Fill model info
        model_info_.name = std::filesystem::path(config.model_path).stem().string();
        model_info_.version = "1.0.0";
        model_info_.runtime = "onnxruntime";
        model_info_.precision = "fp32";
        model_info_.input_shape = pimpl_->input_shapes_.empty() ? std::vector<int64_t>() : pimpl_->input_shapes_[0];
        model_info_.output_names = std::vector<std::string>(pimpl_->output_names_.begin(), pimpl_->output_names_.end());
        model_info_.output_shapes = pimpl_->output_shapes_;
        model_info_.engine_size_bytes = std::filesystem::file_size(config.model_path);
        
        execution_provider_ = pimpl_->execution_provider_;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "ONNX Runtime load error: " << e.what() << std::endl;
        return false;
    }
}

bool ONNXRunner::infer(const float* input_data, size_t input_size,
                       std::vector<float*>& outputs, std::vector<size_t>& output_sizes,
                       float& latency_ms) {
    if (!session_) return false;
    
    try {
        // Prepare input tensor
        std::vector<int64_t> input_shape = pimpl_->input_shapes_[0];
        if (input_shape[0] == -1) input_shape[0] = 1;  // Handle dynamic batch
        
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            pimpl_->allocator_, input_shape.data(), input_shape.size(),
            const_cast<float*>(input_data), input_size * sizeof(float));
        
        // Prepare output tensors
        std::vector<Ort::Value> output_tensors;
        output_tensors.reserve(pimpl_->output_shapes_.size());
        
        for (size_t i = 0; i < pimpl_->output_shapes_.size(); ++i) {
            auto shape = pimpl_->output_shapes_[i];
            if (shape[0] == -1) shape[0] = 1;
            
            size_t output_size = 1;
            for (auto dim : shape) output_size *= std::max<int64_t>(dim, 1);
            
            if (i < outputs.size()) {
                output_tensors.push_back(Ort::Value::CreateTensor<float>(
                    pimpl_->allocator_, shape.data(), shape.size(),
                    outputs[i], output_size * sizeof(float)));
                output_sizes[i] = output_size;
            } else {
                // Allocate new buffer
                float* buffer = new float[output_size];
                output_tensors.push_back(Ort::Value::CreateTensor<float>(
                    pimpl_->allocator_, shape.data(), shape.size(), buffer, output_size * sizeof(float)));
                outputs.push_back(buffer);
                output_sizes.push_back(output_size);
            }
        }
        
        // Run inference
        auto start = std::chrono::high_resolution_clock::now();
        
        session_->Run(Ort::RunOptions(), 
                      pimpl_->input_names_.data(), &input_tensor, 1,
                      pimpl_->output_names_.data(), output_tensors.data(), output_tensors.size());
        
        auto end = std::chrono::high_resolution_clock::now();
        latency_ms = std::chrono::duration<float, std::milli>(end - start).count();
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "ONNX Runtime inference error: " << e.what() << std::endl;
        return false;
    }
}

bool ONNXRunner::infer(const std::vector<std::pair<std::string, const float*>>& inputs,
                       std::vector<std::pair<std::string, float*>>& outputs,
                       float& latency_ms) {
    if (!session_) return false;
    
    try {
        // Build input tensors
        std::vector<Ort::Value> input_tensors;
        input_tensors.reserve(inputs.size());
        
        for (size_t i = 0; i < inputs.size(); ++i) {
            auto shape = pimpl_->input_shapes_[i];
            if (shape[0] == -1) shape[0] = 1;
            
            size_t input_size = 1;
            for (auto dim : shape) input_size *= std::max<int64_t>(dim, 1);
            
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                pimpl_->allocator_, shape.data(), shape.size(),
                const_cast<float*>(inputs[i].second), input_size * sizeof(float)));
        }
        
        // Build output tensors
        std::vector<Ort::Value> output_tensors;
        output_tensors.reserve(outputs.size());
        
        for (size_t i = 0; i < outputs.size(); ++i) {
            auto shape = pimpl_->output_shapes_[i];
            if (shape[0] == -1) shape[0] = 1;
            
            size_t output_size = 1;
            for (auto dim : shape) output_size *= std::max<int64_t>(dim, 1);
            
            output_tensors.push_back(Ort::Value::CreateTensor<float>(
                pimpl_->allocator_, shape.data(), shape.size(),
                outputs[i].second, output_size * sizeof(float)));
        }
        
        // Get input/output names
        std::vector<const char*> input_names, output_names;
        for (const auto& in : inputs) input_names.push_back(in.first.c_str());
        for (const auto& out : outputs) output_names.push_back(out.first.c_str());
        
        // Run inference
        auto start = std::chrono::high_resolution_clock::now();
        
        session_->Run(Ort::RunOptions(),
                      input_names.data(), input_tensors.data(), input_tensors.size(),
                      output_names.data(), output_tensors.data(), output_tensors.size());
        
        auto end = std::chrono::high_resolution_clock::now();
        latency_ms = std::chrono::duration<float, std::milli>(end - start).count();
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "ONNX Runtime inference error: " << e.what() << std::endl;
        return false;
    }
}

ModelInfo ONNXRunner::getModelInfo() const {
    return model_info_;
}

std::vector<ONNXRunner::TensorInfo> ONNXRunner::getInputInfos() const {
    return input_infos_;
}

std::vector<ONNXRunner::TensorInfo> ONNXRunner::getOutputInfos() const {
    return output_infos_;
}

void ONNXRunner::warmup(int iterations) {
    if (!session_) return;
    
    // Create dummy input
    size_t input_size = 1;
    for (auto dim : pimpl_->input_shapes_[0]) input_size *= std::max<int64_t>(dim, 1);
    std::vector<float> dummy_input(input_size, 0.0f);
    
    std::vector<float*> outputs;
    std::vector<size_t> output_sizes;
    float latency_ms;
    
    for (int i = 0; i < iterations; ++i) {
        infer(dummy_input.data(), dummy_input.size(), outputs, output_sizes, latency_ms);
    }
}

std::string ONNXRunner::getExecutionProvider() const {
    return execution_provider_;
}

#else  // HAVE_ONNXRUNTIME not defined

ONNXRunner::ONNXRunner() = default;
ONNXRunner::~ONNXRunner() = default;
ONNXRunner::ONNXRunner(ONNXRunner&&) noexcept = default;
ONNXRunner& ONNXRunner::operator=(ONNXRunner&&) noexcept = default;

bool ONNXRunner::loadModel(const ONNXRunnerConfig&) {
    return false;
}

bool ONNXRunner::infer(const float*, size_t, std::vector<float*>&, std::vector<size_t>&, float&) {
    return false;
}

bool ONNXRunner::infer(const std::vector<std::pair<std::string, const float*>>&,
                       std::vector<std::pair<std::string, float*>>&, float&) {
    return false;
}

ModelInfo ONNXRunner::getModelInfo() const {
    return ModelInfo{};
}

std::vector<ONNXRunner::TensorInfo> ONNXRunner::getInputInfos() const {
    return {};
}

std::vector<ONNXRunner::TensorInfo> ONNXRunner::getOutputInfos() const {
    return {};
}

void ONNXRunner::warmup(int) {}
std::string ONNXRunner::getExecutionProvider() const { return "N/A"; }

#endif  // HAVE_ONNXRUNTIME

}  // namespace swarm::ai