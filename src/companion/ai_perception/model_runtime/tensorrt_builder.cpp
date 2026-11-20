#include "tensorrt_builder.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>

// TensorRT headers
#ifdef HAVE_TENSORRT
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#endif

namespace swarm::ai {

#ifdef HAVE_TENSORRT

using namespace nvinfer1;

// Logger for TensorRT
class TRTLogger : public ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::string prefix;
            switch (severity) {
                case Severity::kINTERNAL_ERROR: prefix = "[TRT INTERNAL_ERROR] "; break;
                case Severity::kERROR: prefix = "[TRT ERROR] "; break;
                case Severity::kWARNING: prefix = "[TRT WARNING] "; break;
                case Severity::kINFO: prefix = "[TRT INFO] "; break;
                case Severity::kVERBOSE: prefix = "[TRT VERBOSE] "; break;
            }
            std::cout << prefix << msg << std::endl;
        }
    }
} g_trt_logger;

// Calibrator implementation
class Int8CalibratorImpl : public IInt8Calibrator {
public:
    Int8CalibratorImpl(const std::string& data_dir,
                       const std::string& cache_file,
                       int batch_size,
                       int num_batches,
                       const std::vector<int64_t>& input_shape,
                       const std::string& input_name)
        : data_dir_(data_dir), cache_file_(cache_file), batch_size_(batch_size),
          num_batches_(num_batches), input_shape_(input_shape), input_name_(input_name) {
        // Find image files
        for (const auto& entry : std::filesystem::directory_iterator(data_dir_)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
                    image_files_.push_back(entry.path().string());
                }
            }
        }
        
        if (image_files_.empty()) {
            throw std::runtime_error("No calibration images found in " + data_dir_);
        }
        
        // Calculate buffer size
        size_t elements = 1;
        for (auto dim : input_shape_) elements *= dim;
        batch_buffer_.resize(elements * batch_size_);
        
        // Load cache if exists
        std::ifstream cache(cache_file_, std::ios::binary);
        if (cache) {
            cache.seekg(0, std::ios::end);
            size_t size = cache.tellg();
            cache.seekg(0, std::ios::beg);
            calibration_cache_.resize(size);
            cache.read(calibration_cache_.data(), size);
        }
    }
    
    int getBatchSize() const override { return batch_size_; }
    
    bool getBatch(void* bindings[], const char* names[], int nbBindings) override {
        if (current_batch_ >= num_batches_) return false;
        
        if (!loadNextBatch()) return false;
        
        // Copy to device
        void* device_ptr = bindings[0];
        cudaMemcpy(device_ptr, batch_buffer_.data(), 
                   batch_buffer_.size() * sizeof(float), cudaMemcpyHostToDevice);
        
        current_batch_++;
        return true;
    }
    
    const void* readCalibrationCache(size_t& length) override {
        if (calibration_cache_.empty()) {
            length = 0;
            return nullptr;
        }
        length = calibration_cache_.size();
        return calibration_cache_.data();
    }
    
    void writeCalibrationCache(const void* ptr, size_t length) override {
        calibration_cache_.assign(static_cast<const char*>(ptr), 
                                  static_cast<const char*>(ptr) + length);
        std::ofstream cache(cache_file_, std::ios::binary);
        cache.write(calibration_cache_.data(), calibration_cache_.size());
    }
    
private:
    bool loadNextBatch() {
        for (int i = 0; i < batch_size_; ++i) {
            size_t idx = (current_batch_ * batch_size_ + i) % image_files_.size();
            preprocessImage(image_files_[idx], &batch_buffer_[i * getElementsPerSample()]);
        }
        return true;
    }
    
    size_t getElementsPerSample() const {
        size_t elements = 1;
        for (auto dim : input_shape_) elements *= dim;
        return elements;
    }
    
    void preprocessImage(const std::string& image_path, float* output) {
        cv::Mat img = cv::imread(image_path);
        if (img.empty()) {
            std::fill_n(output, getElementsPerSample(), 0.0f);
            return;
        }
        
        // Resize to input shape (assuming NCHW)
        int target_h = input_shape_[2];
        int target_w = input_shape_[3];
        cv::resize(img, img, cv::Size(target_w, target_h));
        
        // Convert to float and normalize (ImageNet stats)
        img.convertTo(img, CV_32F, 1.0f / 255.0f);
        
        // CHW format
        std::vector<cv::Mat> channels(3);
        cv::split(img, channels);
        
        // Normalize
        float mean[3] = {0.485f, 0.456f, 0.406f};
        float std[3] = {0.229f, 0.224f, 0.225f};
        
        for (int c = 0; c < 3; ++c) {
            channels[c] = (channels[c] - mean[c]) / std[c];
            float* dst = output + c * target_h * target_w;
            std::memcpy(dst, channels[c].data, target_h * target_w * sizeof(float));
        }
    }
    
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
};

// TensorRTBuilder implementation
class TensorRTBuilder::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    BuildResult build(const TensorRTBuilderConfig& config, 
                      std::unique_ptr<IInt8Calibrator> calibrator = nullptr) {
        BuildResult result;
        auto start = std::chrono::high_resolution_clock::now();
        
        // Create builder
        IBuilder* builder = createInferBuilder(g_trt_logger);
        if (!builder) {
            result.error_message = "Failed to create TensorRT builder";
            return result;
        }
        
        // Create network
        auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        INetworkDefinition* network = builder->createNetworkV2(explicitBatch);
        if (!network) {
            result.error_message = "Failed to create network";
            builder->destroy();
            return result;
        }
        
        // Parse ONNX
        IParser* parser = createParser(*network, g_trt_logger);
        if (!parser) {
            result.error_message = "Failed to create ONNX parser";
            network->destroy();
            builder->destroy();
            return result;
        }
        
        if (!parser->parseFromFile(config.onnx_model_path.c_str(), 
                                   static_cast<int>(ILogger::Severity::kWARNING))) {
            result.error_message = "Failed to parse ONNX file: " + config.onnx_model_path;
            parser->destroy();
            network->destroy();
            builder->destroy();
            return result;
        }
        
        // Print network info
        if (config.verbose) {
            std::cout << "Network inputs: " << network->getNbInputs() << std::endl;
            std::cout << "Network outputs: " << network->getNbOutputs() << std::endl;
            for (int i = 0; i < network->getNbInputs(); ++i) {
                ITensor* input = network->getInput(i);
                std::cout << "  Input " << i << ": " << input->getName() 
                          << ", shape: " << dimsToString(input->getDimensions()) << std::endl;
            }
            for (int i = 0; i < network->getNbOutputs(); ++i) {
                ITensor* output = network->getOutput(i);
                std::cout << "  Output " << i << ": " << output->getName()
                          << ", shape: " << dimsToString(output->getDimensions()) << std::endl;
            }
        }
        
        // Create builder config
        IBuilderConfig* builder_config = builder->createBuilderConfig();
        if (!builder_config) {
            result.error_message = "Failed to create builder config";
            parser->destroy();
            network->destroy();
            builder->destroy();
            return result;
        }
        
        // Set workspace size
        builder_config->setMaxWorkspaceSize(config.max_workspace_size_mb * 1024 * 1024);
        
        // Set precision
        if (config.enable_fp16 && builder->platformHasFastFp16()) {
            builder_config->setFlag(BuilderFlag::kFP16);
            result.precision_used = "fp16";
        } else {
            result.precision_used = "fp32";
        }
        
        // INT8 calibration
        if (config.enable_int8 && calibrator) {
            if (builder->platformHasFastInt8()) {
                builder_config->setFlag(BuilderFlag::kINT8);
                builder_config->setInt8Calibrator(calibrator.get());
                result.precision_used = "int8";
            } else {
                std::cout << "WARNING: INT8 not supported on this platform, falling back to FP16" << std::endl;
                if (builder->platformHasFastFp16()) {
                    builder_config->setFlag(BuilderFlag::kFP16);
                    result.precision_used = "fp16";
                } else {
                    result.precision_used = "fp32";
                }
            }
        }
        
        // DLA core
        if (config.dla_core >= 0) {
            builder_config->setDefaultDeviceType(DeviceType::kDLA);
            builder_config->setDLACore(config.dla_core);
            builder_config->setFlag(BuilderFlag::kGPU_FALLBACK);
        }
        
        // Strict type constraints
        if (config.strict_type_constraints) {
            builder_config->setFlag(BuilderFlag::kSTRICT_TYPES);
        }
        
        // Build engine
        ICudaEngine* engine = builder->buildEngineWithConfig(*network, *builder_config);
        if (!engine) {
            result.error_message = "Failed to build engine";
            builder_config->destroy();
            parser->destroy();
            network->destroy();
            builder->destroy();
            return result;
        }
        
        // Serialize engine
        IHostMemory* serialized_engine = engine->serialize();
        if (!serialized_engine) {
            result.error_message = "Failed to serialize engine";
            engine->destroy();
            builder_config->destroy();
            parser->destroy();
            network->destroy();
            builder->destroy();
            return result;
        }
        
        // Write to file
        std::ofstream outfile(config.engine_output_path, std::ios::binary);
        if (!outfile) {
            result.error_message = "Failed to open output file: " + config.engine_output_path;
            serialized_engine->destroy();
            engine->destroy();
            builder_config->destroy();
            parser->destroy();
            network->destroy();
            builder->destroy();
            return result;
        }
        
        outfile.write(static_cast<const char*>(serialized_engine->data()), 
                      serialized_engine->size());
        outfile.close();
        
        // Fill result
        result.success = true;
        result.engine_path = config.engine_output_path;
        result.engine_size_bytes = serialized_engine->size();
        result.max_batch_size = config.max_batch_size;
        
        // Get input/output info
        for (int i = 0; i < engine->getNbBindings(); ++i) {
            if (engine->bindingIsInput(i)) {
                result.input_shape = dimsToVector(engine->getBindingDimensions(i));
            } else {
                result.output_names.push_back(engine->getBindingName(i));
                result.output_shapes.push_back(dimsToVector(engine->getBindingDimensions(i)));
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        result.build_time_seconds = std::chrono::duration<float>(end - start).count();
        
        // Cleanup
        serialized_engine->destroy();
        engine->destroy();
        builder_config->destroy();
        parser->destroy();
        network->destroy();
        builder->destroy();
        
        return result;
    }
    
private:
    std::string dimsToString(const Dims& dims) {
        std::string s = "[";
        for (int i = 0; i < dims.nbDims; ++i) {
            if (i > 0) s += ", ";
            s += std::to_string(dims.d[i]);
        }
        s += "]";
        return s;
    }
    
    std::vector<int64_t> dimsToVector(const Dims& dims) {
        std::vector<int64_t> v;
        for (int i = 0; i < dims.nbDims; ++i) {
            v.push_back(dims.d[i]);
        }
        return v;
    }
};

TensorRTBuilder::TensorRTBuilder() : pimpl_(std::make_unique<Impl>()) {}
TensorRTBuilder::~TensorRTBuilder() = default;
TensorRTBuilder::TensorRTBuilder(TensorRTBuilder&&) noexcept = default;
TensorRTBuilder& TensorRTBuilder::operator=(TensorRTBuilder&&) noexcept = default;

BuildResult TensorRTBuilder::buildEngine(const TensorRTBuilderConfig& config) {
    return pimpl_->build(config, nullptr);
}

BuildResult TensorRTBuilder::buildEngineInt8(const TensorRTBuilderConfig& config,
                                              std::unique_ptr<IInt8Calibrator> calibrator) {
    return pimpl_->build(config, std::move(calibrator));
}

bool TensorRTBuilder::validateEngine(const std::string& engine_path,
                                     const std::vector<int64_t>& expected_input_shape) {
    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file) return false;
    
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    
    IRuntime* runtime = createInferRuntime(g_trt_logger);
    ICudaEngine* engine = runtime->deserializeCudaEngine(buffer.data(), size, nullptr);
    
    if (!engine) {
        runtime->destroy();
        return false;
    }
    
    bool valid = true;
    if (!expected_input_shape.empty()) {
        for (int i = 0; i < engine->getNbBindings(); ++i) {
            if (engine->bindingIsInput(i)) {
                auto dims = engine->getBindingDimensions(i);
                std::vector<int64_t> actual;
                for (int j = 0; j < dims.nbDims; ++j) actual.push_back(dims.d[j]);
                if (actual != expected_input_shape) {
                    valid = false;
                }
                break;
            }
        }
    }
    
    engine->destroy();
    runtime->destroy();
    return valid;
}

ModelInfo TensorRTBuilder::getEngineInfo(const std::string& engine_path) {
    ModelInfo info;
    
    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file) return info;
    
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    
    IRuntime* runtime = createInferRuntime(g_trt_logger);
    ICudaEngine* engine = runtime->deserializeCudaEngine(buffer.data(), size, nullptr);
    
    if (!engine) {
        runtime->destroy();
        return info;
    }
    
    info.engine_size_bytes = size;
    
    for (int i = 0; i < engine->getNbBindings(); ++i) {
        if (engine->bindingIsInput(i)) {
            info.input_shape = dimsToVector(engine->getBindingDimensions(i));
        } else {
            info.output_names.push_back(engine->getBindingName(i));
            info.output_shapes.push_back(dimsToVector(engine->getBindingDimensions(i)));
        }
    }
    
    engine->destroy();
    runtime->destroy();
    return info;
}

bool TensorRTBuilder::parsePrecision(const std::string& precision_str,
                                     bool& out_fp16, bool& out_int8) {
    std::string p = precision_str;
    std::transform(p.begin(), p.end(), p.begin(), ::tolower);
    
    if (p == "fp32" || p == "float32") {
        out_fp16 = false; out_int8 = false; return true;
    }
    if (p == "fp16" || p == "float16" || p == "half") {
        out_fp16 = true; out_int8 = false; return true;
    }
    if (p == "int8") {
        out_fp16 = true; out_int8 = true; return true;  // INT8 requires FP16
    }
    return false;
}

std::string TensorRTBuilder::getTensorRTVersion() {
    return std::to_string(NV_TENSORRT_VERSION_MAJOR) + "." +
           std::to_string(NV_TENSORRT_VERSION_MINOR) + "." +
           std::to_string(NV_TENSORRT_VERSION_PATCH);
}

bool TensorRTBuilder::checkGPUCompatibility(int gpu_device_id) {
    cudaDeviceProp prop;
    cudaError_t err = cudaGetDeviceProperties(&prop, gpu_device_id);
    if (err != cudaSuccess) return false;
    
    // Check compute capability >= 5.3 for FP16, >= 7.0 for INT8
    return (prop.major > 5) || (prop.major == 5 && prop.minor >= 3);
}

// ImageDirectoryCalibrator implementation
ImageDirectoryCalibrator::ImageDirectoryCalibrator(
    const std::string& data_dir, const std::string& cache_file,
    int batch_size, int num_batches,
    const std::vector<int64_t>& input_shape, const std::string& input_name)
    : data_dir_(data_dir), cache_file_(cache_file), batch_size_(batch_size),
      num_batches_(num_batches), input_shape_(input_shape), input_name_(input_name) {
    
    for (const auto& entry : std::filesystem::directory_iterator(data_dir_)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
                image_files_.push_back(entry.path().string());
            }
        }
    }
    
    if (image_files_.empty()) {
        throw std::runtime_error("No calibration images found in " + data_dir_);
    }
    
    size_t elements = 1;
    for (auto dim : input_shape_) elements *= dim;
    batch_buffer_.resize(elements * batch_size_);
    
    std::ifstream cache(cache_file_, std::ios::binary);
    if (cache) {
        cache.seekg(0, std::ios::end);
        size_t size = cache.tellg();
        cache.seekg(0, std::ios::beg);
        calibration_cache_.resize(size);
        cache.read(calibration_cache_.data(), size);
    }
}

ImageDirectoryCalibrator::~ImageDirectoryCalibrator() = default;

int ImageDirectoryCalibrator::getBatchSize() const { return batch_size_; }

bool ImageDirectoryCalibrator::getBatch(void* bindings[], const char* names[], int nbBindings) {
    if (current_batch_ >= num_batches_) return false;
    
    for (int i = 0; i < batch_size_; ++i) {
        size_t idx = (current_batch_ * batch_size_ + i) % image_files_.size();
        preprocessImage(image_files_[idx], &batch_buffer_[i * getElementsPerSample()]);
    }
    
    cudaMemcpy(bindings[0], batch_buffer_.data(),
               batch_buffer_.size() * sizeof(float), cudaMemcpyHostToDevice);
    
    current_batch_++;
    return true;
}

const void* ImageDirectoryCalibrator::readCalibrationCache(size_t& length) {
    if (calibration_cache_.empty()) {
        length = 0;
        return nullptr;
    }
    length = calibration_cache_.size();
    return calibration_cache_.data();
}

void ImageDirectoryCalibrator::writeCalibrationCache(const void* ptr, size_t length) {
    calibration_cache_.assign(static_cast<const char*>(ptr),
                              static_cast<const char*>(ptr) + length);
    std::ofstream cache(cache_file_, std::ios::binary);
    cache.write(calibration_cache_.data(), calibration_cache_.size());
}

size_t ImageDirectoryCalibrator::getElementsPerSample() const {
    size_t elements = 1;
    for (auto dim : input_shape_) elements *= dim;
    return elements;
}

void ImageDirectoryCalibrator::preprocessImage(const std::string& image_path, float* output) {
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::fill_n(output, getElementsPerSample(), 0.0f);
        return;
    }
    
    int target_h = input_shape_[2];
    int target_w = input_shape_[3];
    cv::resize(img, img, cv::Size(target_w, target_h));
    img.convertTo(img, CV_32F, 1.0f / 255.0f);
    
    std::vector<cv::Mat> channels(3);
    cv::split(img, channels);
    
    float mean[3] = {0.485f, 0.456f, 0.406f};
    float std[3] = {0.229f, 0.224f, 0.225f};
    
    for (int c = 0; c < 3; ++c) {
        channels[c] = (channels[c] - mean[c]) / std[c];
        float* dst = output + c * target_h * target_w;
        std::memcpy(dst, channels[c].data, target_h * target_w * sizeof(float));
    }
}

#else  // HAVE_TENSORRT not defined

// Stub implementations when TensorRT is not available
TensorRTBuilder::TensorRTBuilder() {}
TensorRTBuilder::~TensorRTBuilder() = default;
TensorRTBuilder::TensorRTBuilder(TensorRTBuilder&&) noexcept = default;
TensorRTBuilder& TensorRTBuilder::operator=(TensorRTBuilder&&) noexcept = default;

BuildResult TensorRTBuilder::buildEngine(const TensorRTBuilderConfig& config) {
    BuildResult result;
    result.success = false;
    result.error_message = "TensorRT not available (compile with HAVE_TENSORRT=1)";
    return result;
}

BuildResult TensorRTBuilder::buildEngineInt8(const TensorRTBuilderConfig& config,
                                              std::unique_ptr<IInt8Calibrator>) {
    return buildEngine(config);
}

bool TensorRTBuilder::validateEngine(const std::string&, const std::vector<int64_t>&) {
    return false;
}

ModelInfo TensorRTBuilder::getEngineInfo(const std::string&) {
    return ModelInfo{};
}

bool TensorRTBuilder::parsePrecision(const std::string&, bool&, bool&) {
    return false;
}

std::string TensorRTBuilder::getTensorRTVersion() {
    return "N/A";
}

bool TensorRTBuilder::checkGPUCompatibility(int) {
    return false;
}

ImageDirectoryCalibrator::ImageDirectoryCalibrator(const std::string&, const std::string&,
                                                    int, int, const std::vector<int64_t>&, const std::string&) {}
ImageDirectoryCalibrator::~ImageDirectoryCalibrator() = default;
int ImageDirectoryCalibrator::getBatchSize() const { return 0; }
bool ImageDirectoryCalibrator::getBatch(void*[], const char*[], int) { return false; }
const void* ImageDirectoryCalibrator::readCalibrationCache(size_t& length) { length = 0; return nullptr; }
void ImageDirectoryCalibrator::writeCalibrationCache(const void*, size_t) {}

#endif  // HAVE_TENSORRT

}  // namespace swarm::ai