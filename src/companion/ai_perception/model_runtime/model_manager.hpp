#ifndef MODEL_MANAGER_HPP
#define MODEL_MANAGER_HPP

#include "ai_inference.hpp"
#include "inference_engine.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <functional>

namespace swarm::ai {

struct ModelVersion {
    std::string version;
    std::string model_path;
    std::string checksum;  // SHA256
    uint64_t uploaded_time_us;
    uint64_t activated_time_us = 0;
    bool is_active = false;
    bool is_staged = false;
    std::string runtime;  // "tensorrt", "onnxruntime"
    std::string precision;  // "fp32", "fp16", "int8"
    size_t size_bytes = 0;
    std::map<std::string, std::string> metadata;
};

struct ModelDeployment {
    InferenceType type;
    std::string current_version;
    std::string staged_version;
    std::map<std::string, ModelVersion> versions;  // version -> info
    std::string active_runtime;
    std::string active_precision;
};

struct OTAUpdateConfig {
    std::string download_url_base;
    std::string staging_dir;
    int max_versions_to_keep = 5;
    bool auto_activate = false;
    bool verify_checksum = true;
    bool require_signature = true;
    std::string public_key_path;
    int download_timeout_s = 300;
    int verification_timeout_s = 60;
};

struct DeploymentResult {
    bool success = false;
    std::string error_message;
    std::string version;
    uint64_t deployment_time_us = 0;
};

class ModelManager {
public:
    ModelManager();
    ~ModelManager();
    
    ModelManager(const ModelManager&) = delete;
    ModelManager& operator=(const ModelManager&) = delete;
    ModelManager(ModelManager&&) noexcept;
    ModelManager& operator=(ModelManager&&) noexcept;
    
    // Initialize
    bool initialize(const OTAUpdateConfig& config, InferenceEngine* inference_engine);
    
    // Register a model version locally
    bool registerModelVersion(InferenceType type, const ModelVersion& version);
    
    // Get current active version
    std::optional<ModelVersion> getActiveVersion(InferenceType type) const;
    
    // Get all versions for a type
    std::vector<ModelVersion> getAllVersions(InferenceType type) const;
    
    // Stage a new version (A/B deployment)
    DeploymentResult stageVersion(InferenceType type, const std::string& version);
    
    // Activate staged version (switch A/B)
    DeploymentResult activateStagedVersion(InferenceType type);
    
    // Rollback to previous version
    DeploymentResult rollbackVersion(InferenceType type);
    
    // Download and stage from remote (OTA)
    DeploymentResult downloadAndStage(InferenceType type, const std::string& version,
                                      const std::string& remote_url);
    
    // Verify model integrity
    bool verifyModel(const std::string& model_path, const std::string& expected_checksum);
    
    // Get deployment status
    ModelDeployment getDeploymentStatus(InferenceType type) const;
    
    // List all deployments
    std::vector<ModelDeployment> listDeployments() const;
    
    // Cleanup old versions
    int cleanupOldVersions(InferenceType type, int keep_count = -1);
    
    // Export model registry
    std::string exportRegistry() const;
    
    // Import model registry
    bool importRegistry(const std::string& json);
    
    // Callbacks
    using DeploymentCallback = std::function<void(InferenceType, const DeploymentResult&)>;
    void setDeploymentCallback(DeploymentCallback cb) { deployment_callback_ = cb; }
    
    using ProgressCallback = std::function<void(InferenceType, const std::string&, float progress)>;
    void setProgressCallback(ProgressCallback cb) { progress_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    OTAUpdateConfig config_;
    InferenceEngine* inference_engine_ = nullptr;
    
    std::function<void(InferenceType, const DeploymentResult&)> deployment_callback_;
    std::function<void(InferenceType, const std::string&, float)> progress_callback_;
};

}  // namespace swarm::ai

#endif  // MODEL_MANAGER_HPP