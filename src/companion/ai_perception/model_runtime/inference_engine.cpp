#include "inference_engine.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>

namespace swarm::ai {

class InferenceEngine::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    struct ModelRuntime {
        std::unique_ptr<TensorRTEngine> trt_engine;
        std::unique_ptr<ONNXRunner> ort_runner;
        std::string current_runtime;  // "tensorrt" or "onnxruntime"
        ModelInfo model_info;
        InferenceStats stats;
        bool loaded = false;
    };
    
    ModelRuntime obstacle_model;
    ModelRuntime landing_model;
    ModelRuntime fault_model;
    ModelRuntime anomaly_model;
    
    InferenceEngineConfig config_;
    
    bool loadModel(ModelRuntime& model, const std::string& model_path, 
                   InferenceType type, const std::string& runtime) {
        if (runtime == "tensorrt") {
            std::string engine_path = model_path;
            if (engine_path.find(".onnx") != std::string::npos) {
                engine_path.replace(engine_path.find(".onnx"), 5, ".engine");
            }
            
            // Try to load existing engine
            if (std::filesystem::exists(engine_path)) {
                model.trt_engine = std::make_unique<TensorRTEngine>();
                if (model.trt_engine->loadEngine(engine_path)) {
                    model.trt_engine->createContext();
                    model.current_runtime = "tensorrt";
                    model.model_info = model.trt_engine->getModelInfo();
                    model.loaded = true;
                    return true;
                }
            }
            
            // Build engine from ONNX
            if (std::filesystem::exists(model_path)) {
                TensorRTBuilder builder;
                TensorRTBuilderConfig trt_config;
                trt_config.onnx_model_path = model_path;
                trt_config.engine_output_path = engine_path;
                trt_config.precision = config_.preferred_precision;
                trt_config.max_workspace_size_mb = config_.max_workspace_size_mb;
                trt_config.enable_fp16 = config_.enable_fp16;
                trt_config.enable_int8 = config_.enable_int8;
                trt_config.calibration_cache_path = config_.calibration_cache_dir + "/" + 
                    std::filesystem::path(model_path).stem().string() + ".calib";
                trt_config.calibration_data_path = config_.calibration_data_dir;
                trt_config.dla_core = config_.dla_core;
                trt_config.verbose = config_.enable_profiling;
                
                // Set input shape based on type
                if (type == InferenceType::FAULT) {
                    trt_config.input_shape = {1, 10, 128};  // Sequence: 10 timesteps, 128 features
                } else {
                    trt_config.input_shape = {1, 3, config_.input_height, config_.input_width};
                }
                
                auto result = builder.buildEngine(trt_config);
                if (result.success) {
                    model.trt_engine = std::make_unique<TensorRTEngine>();
                    if (model.trt_engine->loadEngine(engine_path)) {
                        model.trt_engine->createContext();
                        model.current_runtime = "tensorrt";
                        model.model_info = model.trt_engine->getModelInfo();
                        model.loaded = true;
                        return true;
                    }
                }
            }
        }
        
        // Fallback to ONNX Runtime
        if (std::filesystem::exists(model_path)) {
            model.ort_runner = std::make_unique<ONNXRunner>();
            ONNXRunnerConfig ort_config;
            ort_config.model_path = model_path;
            ort_config.execution_provider = runtime == "tensorrt" ? "cuda" : runtime;
            ort_config.device_id = config_.ort_device_id;
            ort_config.intra_op_threads = config_.ort_intra_op_threads;
            ort_config.inter_op_threads = config_.ort_inter_op_threads;
            
            if (type == InferenceType::FAULT) {
                ort_config.input_shape = {1, 10, 128};
            } else {
                ort_config.input_shape = {1, 3, config_.input_height, config_.input_width};
            }
            
            if (model.ort_runner->loadModel(ort_config)) {
                model.current_runtime = "onnxruntime";
                model.model_info = model.ort_runner->getModelInfo();
                model.loaded = true;
                return true;
            }
        }
        
        return false;
    }
    
    bool runInference(ModelRuntime& model, const std::vector<float>& input_tensor,
                      const std::vector<int64_t>& input_shape,
                      std::vector<float*>& outputs, std::vector<size_t>& output_sizes,
                      float& latency_ms, bool& used_fallback) {
        if (!model.loaded) return false;
        
        if (model.current_runtime == "tensorrt" && model.trt_engine) {
            // Convert input_shape to bindings
            std::vector<const void*> inputs = {input_tensor.data()};
            std::vector<void*> output_ptrs(outputs.size());
            for (size_t i = 0; i < outputs.size(); ++i) {
                output_ptrs[i] = outputs[i];
            }
            
            if (model.trt_engine->infer(inputs, output_ptrs, latency_ms)) {
                used_fallback = false;
                return true;
            }
            
            // Fallback to ONNX Runtime
            if (model.ort_runner) {
                std::cout << "TensorRT inference failed, falling back to ONNX Runtime" << std::endl;
            }
        }
        
        if (model.ort_runner) {
            if (model.ort_runner->infer(input_tensor.data(), input_tensor.size(),
                                       outputs, output_sizes, latency_ms)) {
                used_fallback = (model.current_runtime != "onnxruntime");
                model.current_runtime = "onnxruntime";
                return true;
            }
        }
        
        return false;
    }
    
    std::vector<Detection> postprocessYOLO(const float* output, 
                                           const std::vector<int64_t>& output_shape,
                                           const PreprocessResult& preprocess_info,
                                           const std::vector<std::string>& class_names,
                                           float conf_threshold, float nms_threshold) {
        // YOLOv8 output format: [batch, num_boxes, 85] where 85 = 4 (bbox) + 1 (obj_conf) + 80 (classes)
        // Or for YOLOv8-seg: [batch, num_masks, ...]
        
        std::vector<Detection> detections;
        
        if (output_shape.size() != 3) return detections;
        
        int num_boxes = output_shape[1];
        int num_classes = output_shape[2] - 5;  // 4 bbox + 1 obj_conf + num_classes
        
        // Iterate through boxes
        for (int i = 0; i < num_boxes; ++i) {
            const float* box_data = output + i * output_shape[2];
            
            float obj_conf = box_data[4];
            if (obj_conf < conf_threshold) continue;
            
            // Find best class
            float max_class_conf = 0.0f;
            int best_class = -1;
            for (int c = 0; c < num_classes; ++c) {
                float cls_conf = box_data[5 + c] * obj_conf;
                if (cls_conf > max_class_conf) {
                    max_class_conf = cls_conf;
                    best_class = c;
                }
            }
            
            if (max_class_conf < conf_threshold || best_class < 0) continue;
            
            // Extract bbox (cx, cy, w, h) normalized to input size
            float cx = box_data[0];
            float cy = box_data[1];
            float w = box_data[2];
            float h = box_data[3];
            
            // Convert to xyxy in input coordinates
            float x1 = cx - w / 2;
            float y1 = cy - h / 2;
            float x2 = cx + w / 2;
            float y2 = cy + h / 2;
            
            // Map back to original image coordinates
            float scale = preprocess_info.scale;
            int pad_left = preprocess_info.pad_left;
            int pad_top = preprocess_info.pad_top;
            
            x1 = (x1 - pad_left) / scale;
            y1 = (y1 - pad_top) / scale;
            x2 = (x2 - pad_left) / scale;
            y2 = (y2 - pad_top) / scale;
            
            // Clamp to image bounds
            x1 = std::max(0.0f, x1);
            y1 = std::max(0.0f, y1);
            
            Detection det;
            det.class_id = best_class;
            det.class_name = (best_class < class_names.size()) ? class_names[best_class] : "unknown";
            det.confidence = max_class_conf;
            det.bbox_xyxy = Eigen::Vector4f(x1, y1, x2, y2);
            
            detections.push_back(det);
        }
        
        // NMS
        if (detections.size() > 1 && nms_threshold > 0) {
            std::sort(detections.begin(), detections.end(), 
                     [](const Detection& a, const Detection& b) {
                         return a.confidence > b.confidence;
                     });
            
            std::vector<Detection> filtered;
            for (size_t i = 0; i < detections.size(); ++i) {
                bool keep = true;
                for (size_t j = 0; j < filtered.size(); ++j) {
                    float iou = computeIoU(detections[i].bbox_xyxy, filtered[j].bbox_xyxy);
                    if (iou > nms_threshold) {
                        keep = false;
                        break;
                    }
                }
                if (keep) filtered.push_back(detections[i]);
            }
            detections = filtered;
        }
        
        return detections;
    }
    
    float computeIoU(const Eigen::Vector4f& box1, const Eigen::Vector4f& box2) {
        float x1 = std::max(box1[0], box2[0]);
        float y1 = std::max(box1[1], box2[1]);
        float x2 = std::min(box1[2], box2[2]);
        float y2 = std::min(box1[3], box2[3]);
        
        float inter_area = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
        float box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1]);
        float box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1]);
        float union_area = box1_area + box2_area - inter_area;
        
        return union_area > 0 ? inter_area / union_area : 0.0f;
    }
};

InferenceEngine::InferenceEngine() : pimpl_(std::make_unique<Impl>()) {}
InferenceEngine::~InferenceEngine() = default;
InferenceEngine::InferenceEngine(InferenceEngine&&) noexcept = default;
InferenceEngine& InferenceEngine::operator=(InferenceEngine&&) noexcept = default;

bool InferenceEngine::initialize(const InferenceEngineConfig& config) {
    config_ = config;
    pimpl_->config_ = config;
    
    bool any_loaded = false;
    
    // Load obstacle model
    if (!config.obstacle_model_path.empty()) {
        if (pimpl_->loadModel(pimpl_->obstacle_model, config.obstacle_model_path, 
                              InferenceType::OBSTACLE, config.preferred_runtime)) {
            any_loaded = true;
            std::cout << "Loaded obstacle model: " << config.obstacle_model_path 
                      << " (" << pimpl_->obstacle_model.current_runtime << ")" << std::endl;
        }
    }
    
    // Load landing model
    if (!config.landing_model_path.empty()) {
        if (pimpl_->loadModel(pimpl_->landing_model, config.landing_model_path,
                              InferenceType::LANDING_SITE, config.preferred_runtime)) {
            any_loaded = true;
            std::cout << "Loaded landing model: " << config.landing_model_path
                      << " (" << pimpl_->landing_model.current_runtime << ")" << std::endl;
        }
    }
    
    // Load fault model
    if (!config.fault_model_path.empty()) {
        if (pimpl_->loadModel(pimpl_->fault_model, config.fault_model_path,
                              InferenceType::FAULT, config.preferred_runtime)) {
            any_loaded = true;
            std::cout << "Loaded fault model: " << config.fault_model_path
                      << " (" << pimpl_->fault_model.current_runtime << ")" << std::endl;
        }
    }
    
    // Load anomaly model
    if (!config.anomaly_model_path.empty()) {
        if (pimpl_->loadModel(pimpl_->anomaly_model, config.anomaly_model_path,
                              InferenceType::ANOMALY, config.preferred_runtime)) {
            any_loaded = true;
            std::cout << "Loaded anomaly model: " << config.anomaly_model_path
                      << " (" << pimpl_->anomaly_model.current_runtime << ")" << std::endl;
        }
    }
    
    initialized_ = any_loaded;
    
    if (initialized_ && config.warmup_iterations > 0) {
        warmup();
    }
    
    return initialized_;
}

InferenceResult InferenceEngine::inferObstacle(const cv::Mat& image, uint16_t drone_id, uint64_t timestamp_us) {
    InferenceResult result;
    if (!initialized_ || !pimpl_->obstacle_model.loaded) {
        result.total_ms = -1;
        return result;
    }
    
    auto start_total = std::chrono::high_resolution_clock::now();
    
    // Preprocess
    auto start_pre = std::chrono::high_resolution_clock::now();
    PreprocessResult preprocess = preprocessImage(image);
    auto end_pre = std::chrono::high_resolution_clock::now();
    result.preprocess_ms = std::chrono::duration<float, std::milli>(end_pre - start_pre).count();
    
    // Prepare output buffers
    std::vector<float*> outputs;
    std::vector<size_t> output_sizes;
    
    // For YOLO: typically 1 output [1, num_boxes, 85]
    size_t output_size = 1 * 8400 * 85;  // Max detections * (4+1+80)
    float* output_buffer = new float[output_size];
    outputs.push_back(output_buffer);
    output_sizes.push_back(output_size);
    
    // Inference
    auto start_inf = std::chrono::high_resolution_clock::now();
    float latency_ms = 0;
    bool used_fallback = false;
    
    bool success = pimpl_->runInference(pimpl_->obstacle_model, preprocess.input_tensor,
                                        preprocess.input_shape, outputs, output_sizes,
                                        latency_ms, used_fallback);
    
    auto end_inf = std::chrono::high_resolution_clock::now();
    result.inference_ms = std::chrono::duration<float, std::milli>(end_inf - start_inf).count();
    result.used_fallback = used_fallback;
    result.runtime_used = pimpl_->obstacle_model.current_runtime;
    result.model_name = pimpl_->obstacle_model.model_info.name;
    
    if (!success) {
        result.total_ms = -1;
        delete[] output_buffer;
        if (error_callback_) error_callback_(InferenceType::OBSTACLE, "Inference failed");
        return result;
    }
    
    // Postprocess
    auto start_post = std::chrono::high_resolution_clock::now();
    std::vector<std::string> class_names = {"person", "bicycle", "car", "motorcycle", "airplane",
                                             "bus", "train", "truck", "boat", "traffic light",
                                             "fire hydrant", "stop sign", "parking meter", "bench",
                                             "bird", "cat", "dog", "horse", "sheep", "cow",
                                             "elephant", "bear", "zebra", "giraffe", "backpack",
                                             "umbrella", "handbag", "tie", "suitcase", "frisbee",
                                             "skis", "snowboard", "sports ball", "kite", "baseball bat",
                                             "baseball glove", "skateboard", "surfboard", "tennis racket",
                                             "bottle", "wine glass", "cup", "fork", "knife", "spoon",
                                             "bowl", "banana", "apple", "sandwich", "orange",
                                             "broccoli", "carrot", "hot dog", "pizza", "donut",
                                             "cake", "chair", "couch", "potted plant", "bed",
                                             "dining table", "toilet", "tv", "laptop", "mouse",
                                             "remote", "keyboard", "cell phone", "microwave", "oven",
                                             "toaster", "sink", "refrigerator", "book", "clock",
                                             "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
    
    // Add custom classes for drone obstacles
    class_names.insert(class_names.end(), {"power_line", "tower", "wind_turbine", "crane", "antenna",
                                            "drone", "bird", "wire", "cable", "pole", "fence"});
    
    result.detections = pimpl_->postprocessYOLO(output_buffer, {1, 8400, 85}, preprocess,
                                                class_names, config_.confidence_threshold, config_.nms_threshold);
    
    auto end_post = std::chrono::high_resolution_clock::now();
    result.postprocess_ms = std::chrono::duration<float, std::milli>(end_post - start_post).count();
    
    auto end_total = std::chrono::high_resolution_clock::now();
    result.total_ms = std::chrono::duration<float, std::milli>(end_total - start_total).count();
    
    // Update stats
    pimpl_->obstacle_model.stats.recordInference(result.total_ms, success, used_fallback,
                                                 InferenceType::OBSTACLE, 
                                                 result.detections.empty() ? 0.0f : result.detections[0].confidence);
    
    // Convert to AIInferenceResult and callback
    auto inference_results = detectionsToInferenceResults(result.detections, InferenceType::OBSTACLE,
                                                          drone_id, timestamp_us, result.inference_ms,
                                                          result.model_name, result.runtime_used);
    
    for (const auto& inf : inference_results) {
        if (inference_callback_) inference_callback_(inf);
    }
    
    delete[] output_buffer;
    return result;
}

InferenceResult InferenceEngine::inferLanding(const cv::Mat& image, uint16_t drone_id, uint64_t timestamp_us) {
    InferenceResult result;
    if (!initialized_ || !pimpl_->landing_model.loaded) {
        result.total_ms = -1;
        return result;
    }
    
    auto start_total = std::chrono::high_resolution_clock::now();
    
    // Preprocess
    auto start_pre = std::chrono::high_resolution_clock::now();
    PreprocessResult preprocess = preprocessImage(image);
    auto end_pre = std::chrono::high_resolution_clock::now();
    result.preprocess_ms = std::chrono::duration<float, std::milli>(end_pre - start_pre).count();
    
    // Output buffer for classification
    size_t output_size = 1000;  // 1000 classes for ImageNet-style
    float* output_buffer = new float[output_size];
    std::vector<float*> outputs = {output_buffer};
    std::vector<size_t> output_sizes = {output_size};
    
    // Inference
    auto start_inf = std::chrono::high_resolution_clock::now();
    float latency_ms = 0;
    bool used_fallback = false;
    
    bool success = pimpl_->runInference(pimpl_->landing_model, preprocess.input_tensor,
                                        preprocess.input_shape, outputs, output_sizes,
                                        latency_ms, used_fallback);
    
    auto end_inf = std::chrono::high_resolution_clock::now();
    result.inference_ms = std::chrono::duration<float, std::milli>(end_inf - start_inf).count();
    result.used_fallback = used_fallback;
    result.runtime_used = pimpl_->landing_model.current_runtime;
    result.model_name = pimpl_->landing_model.model_info.name;
    
    if (!success) {
        result.total_ms = -1;
        delete[] output_buffer;
        if (error_callback_) error_callback_(InferenceType::LANDING_SITE, "Inference failed");
        return result;
    }
    
    // Postprocess (simplified - would use actual class labels)
    auto start_post = std::chrono::high_resolution_clock::now();
    result.detections = postprocessLanding(output_buffer, {1, 1000});
    auto end_post = std::chrono::high_resolution_clock::now();
    result.postprocess_ms = std::chrono::duration<float, std::milli>(end_post - start_post).count();
    
    auto end_total = std::chrono::high_resolution_clock::now();
    result.total_ms = std::chrono::duration<float, std::milli>(end_total - start_total).count();
    
    pimpl_->landing_model.stats.recordInference(result.total_ms, success, used_fallback,
                                                InferenceType::LANDING_SITE,
                                                result.detections.empty() ? 0.0f : result.detections[0].confidence);
    
    auto inference_results = detectionsToInferenceResults(result.detections, InferenceType::LANDING_SITE,
                                                          drone_id, timestamp_us, result.inference_ms,
                                                          result.model_name, result.runtime_used);
    
    for (const auto& inf : inference_results) {
        if (inference_callback_) inference_callback_(inf);
    }
    
    delete[] output_buffer;
    return result;
}

InferenceResult InferenceEngine::inferFault(const std::vector<float>& sensor_sequence, 
                                            uint16_t drone_id, uint64_t timestamp_us) {
    InferenceResult result;
    if (!initialized_ || !pimpl_->fault_model.loaded) {
        result.total_ms = -1;
        return result;
    }
    
    auto start_total = std::chrono::high_resolution_clock::now();
    
    // Input is already preprocessed sequence
    auto start_inf = std::chrono::high_resolution_clock::now();
    std::vector<int64_t> input_shape = {1, 10, 128};  // Assuming 10 timesteps, 128 features
    std::vector<float*> outputs;
    std::vector<size_t> output_sizes;
    
    size_t output_size = 10;  // Fault classes
    float* output_buffer = new float[output_size];
    outputs.push_back(output_buffer);
    output_sizes.push_back(output_size);
    
    float latency_ms = 0;
    bool used_fallback = false;
    
    bool success = pimpl_->runInference(pimpl_->fault_model, sensor_sequence,
                                        input_shape, outputs, output_sizes,
                                        latency_ms, used_fallback);
    
    auto end_inf = std::chrono::high_resolution_clock::now();
    result.inference_ms = std::chrono::duration<float, std::milli>(end_inf - start_inf).count();
    result.used_fallback = used_fallback;
    result.runtime_used = pimpl_->fault_model.current_runtime;
    result.model_name = pimpl_->fault_model.model_info.name;
    
    if (!success) {
        result.total_ms = -1;
        delete[] output_buffer;
        if (error_callback_) error_callback_(InferenceType::FAULT, "Inference failed");
        return result;
    }
    
    auto start_post = std::chrono::high_resolution_clock::now();
    result.detections = postprocessFault(output_buffer, {1, 10});
    auto end_post = std::chrono::high_resolution_clock::now();
    result.postprocess_ms = std::chrono::duration<float, std::milli>(end_post - start_post).count();
    
    auto end_total = std::chrono::high_resolution_clock::now();
    result.total_ms = std::chrono::duration<float, std::milli>(end_total - start_total).count();
    
    pimpl_->fault_model.stats.recordInference(result.total_ms, success, used_fallback,
                                              InferenceType::FAULT,
                                              result.detections.empty() ? 0.0f : result.detections[0].confidence);
    
    auto inference_results = detectionsToInferenceResults(result.detections, InferenceType::FAULT,
                                                          drone_id, timestamp_us, result.inference_ms,
                                                          result.model_name, result.runtime_used);
    
    for (const auto& inf : inference_results) {
        if (inference_callback_) inference_callback_(inf);
    }
    
    delete[] output_buffer;
    return result;
}

InferenceResult InferenceEngine::infer(const std::vector<float>& input_tensor,
                                       const std::vector<int64_t>& input_shape,
                                       InferenceType type,
                                       uint16_t drone_id,
                                       uint64_t timestamp_us) {
    InferenceResult result;
    result.total_ms = -1;
    return result;  // Not implemented for generic type
}

PreprocessResult InferenceEngine::preprocessImage(const cv::Mat& image) {
    PreprocessResult result;
    
    int h = image.rows;
    int w = image.cols;
    int target_h = config_.input_height;
    int target_w = config_.input_width;
    
    // Letterbox resize
    float scale = std::min(float(target_w) / w, float(target_h) / h);
    int new_w = int(w * scale);
    int new_h = int(h * scale);
    int pad_left = (target_w - new_w) / 2;
    int pad_top = (target_h - new_h) / 2;
    
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h));
    
    cv::Mat padded = cv::Mat::zeros(target_h, target_w, image.type());
    resized.copyTo(padded(cv::Rect(pad_left, pad_top, new_w, new_h)));
    
    // Normalize and convert to NCHW
    cv::Mat normalized;
    padded.convertTo(normalized, CV_32F, 1.0f / 255.0f);
    
    std::vector<cv::Mat> channels(3);
    cv::split(normalized, channels);
    
    for (int c = 0; c < 3; ++c) {
        channels[c] = (channels[c] - config_.input_mean[c]) / config_.input_std[c];
    }
    
    // Flatten to NCHW
    size_t tensor_size = 3 * target_h * target_w;
    result.input_tensor.resize(tensor_size);
    result.input_shape = {1, 3, target_h, target_w};
    
    for (int c = 0; c < 3; ++c) {
        float* dst = result.input_tensor.data() + c * target_h * target_w;
        std::memcpy(dst, channels[c].data, target_h * target_w * sizeof(float));
    }
    
    result.scale = scale;
    result.pad_left = pad_left;
    result.pad_top = pad_top;
    
    // Transform matrix for coordinate mapping
    result.transform_matrix = Eigen::Matrix3f::Identity();
    result.transform_matrix(0, 0) = 1.0f / scale;
    result.transform_matrix(1, 1) = 1.0f / scale;
    result.transform_matrix(0, 2) = -pad_left / scale;
    result.transform_matrix(1, 2) = -pad_top / scale;
    
    return result;
}

std::vector<Detection> InferenceEngine::postprocessObstacle(const float* output,
                                                            const std::vector<int64_t>& output_shape,
                                                            const PreprocessResult& preprocess_info) {
    // Simplified - would use actual postprocessing
    return pimpl_->postprocessYOLO(output, output_shape, preprocess_info,
                                   {"power_line", "tower", "building", "tree", "bird", "drone",
                                    "aircraft", "vehicle", "person", "wind_turbine", "crane",
                                    "antenna", "wire", "cable", "pole", "fence"},
                                   config_.confidence_threshold, config_.nms_threshold);
}

std::vector<Detection> InferenceEngine::postprocessLanding(const float* output,
                                                           const std::vector<int64_t>& output_shape) {
    std::vector<Detection> detections;
    // Find top class
    int num_classes = output_shape.back();
    float max_conf = 0.0f;
    int best_class = 0;
    
    for (int i = 0; i < num_classes; ++i) {
        if (output[i] > max_conf) {
            max_conf = output[i];
            best_class = i;
        }
    }
    
    if (max_conf > config_.confidence_threshold) {
        Detection det;
        det.class_id = best_class;
        det.class_name = "landing_site_" + std::to_string(best_class);
        det.confidence = max_conf;
        detections.push_back(det);
    }
    
    return detections;
}

std::vector<Detection> InferenceEngine::postprocessFault(const float* output,
                                                         const std::vector<int64_t>& output_shape) {
    std::vector<Detection> detections;
    int num_classes = output_shape.back();
    
    std::vector<std::string> fault_names = {"imu_drift", "gps_spoof", "motor_degrade", 
                                             "battery_imbalance", "sensor_fault", "comm_degrade"};
    
    for (int i = 0; i < num_classes && i < fault_names.size(); ++i) {
        if (output[i] > config_.confidence_threshold) {
            Detection det;
            det.class_id = i;
            det.class_name = fault_names[i];
            det.confidence = output[i];
            detections.push_back(det);
        }
    }
    
    return detections;
}

std::vector<AIInferenceResult> InferenceEngine::detectionsToInferenceResults(
    const std::vector<Detection>& detections,
    InferenceType type,
    uint16_t drone_id,
    uint64_t timestamp_us,
    float inference_ms,
    const std::string& model_name,
    const std::string& runtime) {
    
    std::vector<AIInferenceResult> results;
    
    for (const auto& det : detections) {
        AIInferenceResult inf;
        inf.timestamp_us = timestamp_us;
        inf.drone_id = drone_id;
        inf.type = type;
        inf.class_id = det.class_id;
        inf.class_name = det.class_name;
        inf.confidence = det.confidence;
        
        // Convert bbox to 3D position (simplified)
        inf.position_enu = Eigen::Vector3f(det.bbox_xyxy[0], det.bbox_xyxy[1], 0);
        inf.size = Eigen::Vector3f(det.bbox_xyxy[2] - det.bbox_xyxy[0],
                                   det.bbox_xyxy[3] - det.bbox_xyxy[1], 1.0f);
        inf.orientation = Eigen::Quaternionf::Identity();
        
        inf.metadata.latency_ms = static_cast<int>(inference_ms);
        inf.metadata.model_name = model_name;
        inf.metadata.model_version = "1.0.0";
        inf.metadata.runtime = runtime;
        
        // Type-specific metadata
        if (type == InferenceType::OBSTACLE) {
            inf.metadata.obstacle.threat_level = det.confidence > 0.9 ? ThreatLevel::CRITICAL :
                                                 det.confidence > 0.7 ? ThreatLevel::HIGH :
                                                 det.confidence > 0.5 ? ThreatLevel::MEDIUM : ThreatLevel::LOW;
            inf.metadata.obstacle.requires_immediate_avoidance = (inf.metadata.obstacle.threat_level >= ThreatLevel::HIGH);
            inf.metadata.obstacle.time_to_collision_s = -1.0f;  // Would compute from velocity
        } else if (type == InferenceType::LANDING_SITE) {
            inf.metadata.landing.surface_type = SurfaceType::GRASS;  // Would classify
            inf.metadata.landing.slope_deg = 0.0f;
            inf.metadata.landing.clearance_radius_m = 5.0f;
            inf.metadata.landing.firmness_score = 0.8f;
        } else if (type == InferenceType::FAULT) {
            inf.metadata.fault.fault_type = static_cast<FaultType>(det.class_id);
            inf.metadata.fault.severity = det.confidence > 0.9 ? FaultSeverity::CRITICAL :
                                          det.confidence > 0.7 ? FaultSeverity::WARNING : FaultSeverity::INFO;
            inf.metadata.fault.affected_component = det.class_name;
            inf.metadata.fault.degradation_pct = det.confidence;
            inf.metadata.fault.requires_replan = (inf.metadata.fault.severity >= FaultSeverity::CRITICAL);
            inf.metadata.fault.recommended_action = inf.metadata.fault.requires_replan ? 
                RecommendedAction::RTL : RecommendedAction::DEGRADED;
        }
        
        results.push_back(inf);
    }
    
    return results;
}

ModelInfo InferenceEngine::getObstacleModelInfo() const { return pimpl_->obstacle_model.model_info; }
ModelInfo InferenceEngine::getLandingModelInfo() const { return pimpl_->landing_model.model_info; }
ModelInfo InferenceEngine::getFaultModelInfo() const { return pimpl_->fault_model.model_info; }
ModelInfo InferenceEngine::getAnomalyModelInfo() const { return pimpl_->anomaly_model.model_info; }

InferenceStats InferenceEngine::getStats(InferenceType type) const {
    switch (type) {
        case InferenceType::OBSTACLE: return pimpl_->obstacle_model.stats;
        case InferenceType::LANDING_SITE: return pimpl_->landing_model.stats;
        case InferenceType::FAULT: return pimpl_->fault_model.stats;
        case InferenceType::ANOMALY: return pimpl_->anomaly_model.stats;
    }
    return InferenceStats{};
}

bool InferenceEngine::isInitialized() const { return initialized_; }

std::string InferenceEngine::getCurrentRuntime(InferenceType type) const {
    switch (type) {
        case InferenceType::OBSTACLE: return pimpl_->obstacle_model.current_runtime;
        case InferenceType::LANDING_SITE: return pimpl_->landing_model.current_runtime;
        case InferenceType::FAULT: return pimpl_->fault_model.current_runtime;
        case InferenceType::ANOMALY: return pimpl_->anomaly_model.current_runtime;
    }
    return "unknown";
}

bool InferenceEngine::switchRuntime(InferenceType type, const std::string& runtime) {
    // Would reinitialize model with different runtime
    return false;
}

void InferenceEngine::warmup() {
    // Create dummy input
    cv::Mat dummy(config_.input_height, config_.input_width, CV_8UC3, cv::Scalar(128, 128, 128));
    
    for (int i = 0; i < config_.warmup_iterations; ++i) {
        if (pimpl_->obstacle_model.loaded) inferObstacle(dummy, 0, 0);
        if (pimpl_->landing_model.loaded) inferLanding(dummy, 0, 0);
        // Fault needs sequence data
    }
}

}  // namespace swarm::ai