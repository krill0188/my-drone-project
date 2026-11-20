#include "model_manager.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace swarm::ai {

using json = nlohmann::json;

class ModelManager::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    std::map<InferenceType, ModelDeployment> deployments_;
    std::mutex mutex_;
    OTAUpdateConfig config_;
    InferenceEngine* inference_engine_ = nullptr;
    
    std::string computeSHA256(const std::string& file_path) {
        std::ifstream file(file_path, std::ios::binary);
        if (!file) return "";
        
        EVP_MD_CTX* ctx = EVP_MD_CTX_new();
        EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr);
        
        char buffer[8192];
        while (file.read(buffer, sizeof(buffer)) || file.gcount() > 0) {
            EVP_DigestUpdate(ctx, buffer, file.gcount());
        }
        
        unsigned char hash[EVP_MAX_MD_SIZE];
        unsigned int hash_len;
        EVP_DigestFinal_ex(ctx, hash, &hash_len);
        EVP_MD_CTX_free(ctx);
        
        std::stringstream ss;
        for (unsigned int i = 0; i < hash_len; ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
        }
        return ss.str();
    }
    
    size_t downloadFile(const std::string& url, const std::string& output_path,
                        std::function<void(float)> progress_cb) {
        CURL* curl = curl_easy_init();
        if (!curl) return 0;
        
        FILE* fp = fopen(output_path.c_str(), "wb");
        if (!fp) {
            curl_easy_cleanup(curl);
            return 0;
        }
        
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, fwrite);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, config_.download_timeout_s);
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
        
        curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, 
            [](void* clientp, curl_off_t dltotal, curl_off_t dlnow, 
               curl_off_t ultotal, curl_off_t ulnow) -> int {
                auto cb = static_cast<std::function<void(float)>*>(clientp);
                if (dltotal > 0 && cb && *cb) {
                    (*cb)(static_cast<float>(dlnow) / static_cast<float>(dltotal));
                }
                return 0;
            });
        curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &progress_cb);
        
        CURLcode res = curl_easy_perform(curl);
        long response_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
        
        fclose(fp);
        curl_easy_cleanup(curl);
        
        if (res != CURLE_OK || response_code != 200) {
            std::filesystem::remove(output_path);
            return 0;
        }
        
        return std::filesystem::file_size(output_path);
    }
    
    bool verifySignature(const std::string& file_path, const std::string& sig_path,
                         const std::string& pubkey_path) {
        // Simplified - would use OpenSSL to verify RSA/ECDSA signature
        return true;
    }
    
    DeploymentResult doStageVersion(InferenceType type, const std::string& version) {
        DeploymentResult result;
        auto it = deployments_.find(type);
        if (it == deployments_.end()) {
            result.error_message = "No deployment for this type";
            return result;
        }
        
        auto& deployment = it->second;
        auto ver_it = deployment.versions.find(version);
        if (ver_it == deployment.versions.end()) {
            result.error_message = "Version not found: " + version;
            return result;
        }
        
        // Check if already staged
        if (ver_it->second.is_staged) {
            result.success = true;
            result.version = version;
            return result;
        }
        
        // Unstage current staged
        for (auto& [v, ver] : deployment.versions) {
            if (ver.is_staged) {
                ver.is_staged = false;
            }
        }
        
        // Stage new version
        ver_it->second.is_staged = true;
        deployment.staged_version = version;
        
        result.success = true;
        result.version = version;
        result.deployment_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        return result;
    }
    
    DeploymentResult doActivateVersion(InferenceType type) {
        DeploymentResult result;
        auto it = deployments_.find(type);
        if (it == deployments_.end()) {
            result.error_message = "No deployment for this type";
            return result;
        }
        
        auto& deployment = it->second;
        if (deployment.staged_version.empty()) {
            result.error_message = "No staged version to activate";
            return result;
        }
        
        // Deactivate current
        if (!deployment.current_version.empty()) {
            auto curr_it = deployment.versions.find(deployment.current_version);
            if (curr_it != deployment.versions.end()) {
                curr_it->second.is_active = false;
            }
        }
        
        // Activate staged
        auto staged_it = deployment.versions.find(deployment.staged_version);
        if (staged_it == deployment.versions.end()) {
            result.error_message = "Staged version not found";
            return result;
        }
        
        staged_it->second.is_active = true;
        staged_it->second.activated_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        deployment.current_version = deployment.staged_version;
        deployment.staged_version.clear();
        
        // Update inference engine if available
        if (inference_engine_) {
            // Would reload model with new version
        }
        
        result.success = true;
        result.version = deployment.current_version;
        result.deployment_time_us = staged_it->second.activated_time_us;
        
        return result;
    }
    
    DeploymentResult doRollback(InferenceType type) {
        DeploymentResult result;
        auto it = deployments_.find(type);
        if (it == deployments_.end()) {
            result.error_message = "No deployment for this type";
            return result;
        }
        
        auto& deployment = it->second;
        
        // Find previous active version
        std::string prev_version;
        uint64_t latest_time = 0;
        for (const auto& [v, ver] : deployment.versions) {
            if (v != deployment.current_version && ver.activated_time_us > latest_time) {
                latest_time = ver.activated_time_us;
                prev_version = v;
            }
        }
        
        if (prev_version.empty()) {
            result.error_message = "No previous version to rollback to";
            return result;
        }
        
        // Stage and activate previous
        auto stage_result = doStageVersion(type, prev_version);
        if (!stage_result.success) return stage_result;
        
        return doActivateVersion(type);
    }
};

ModelManager::ModelManager() : pimpl_(std::make_unique<Impl>()) {}
ModelManager::~ModelManager() = default;
ModelManager::ModelManager(ModelManager&&) noexcept = default;
ModelManager& ModelManager::operator=(ModelManager&&) noexcept = default;

bool ModelManager::initialize(const OTAUpdateConfig& config, InferenceEngine* inference_engine) {
    config_ = config;
    pimpl_->config_ = config;
    pimpl_->inference_engine_ = inference_engine;
    
    // Create staging directory
    std::filesystem::create_directories(config.staging_dir);
    
    // Initialize deployments for each type
    for (int t = 0; t < 4; ++t) {
        InferenceType type = static_cast<InferenceType>(t);
        pimpl_->deployments_[type] = ModelDeployment{};
        pimpl_->deployments_[type].type = type;
    }
    
    // Initialize curl
    curl_global_init(CURL_GLOBAL_DEFAULT);
    
    return true;
}

bool ModelManager::registerModelVersion(InferenceType type, const ModelVersion& version) {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    
    auto it = pimpl_->deployments_.find(type);
    if (it == pimpl_->deployments_.end()) return false;
    
    // Verify file exists
    if (!std::filesystem::exists(version.model_path)) return false;
    
    // Compute checksum if not provided
    ModelVersion ver = version;
    if (ver.checksum.empty()) {
        ver.checksum = pimpl_->computeSHA256(ver.model_path);
    }
    
    if (ver.size_bytes == 0) {
        ver.size_bytes = std::filesystem::file_size(ver.model_path);
    }
    
    if (ver.uploaded_time_us == 0) {
        ver.uploaded_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    
    it->second.versions[ver.version] = ver;
    
    // If first version, make it active
    if (it->second.current_version.empty() && !ver.is_staged) {
        it->second.current_version = ver.version;
        it->second.versions[ver.version].is_active = true;
        it->second.versions[ver.version].activated_time_us = ver.uploaded_time_us;
    }
    
    return true;
}

std::optional<ModelVersion> ModelManager::getActiveVersion(InferenceType type) const {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    
    auto it = pimpl_->deployments_.find(type);
    if (it == pimpl_->deployments_.end()) return std::nullopt;
    
    if (it->second.current_version.empty()) return std::nullopt;
    
    auto ver_it = it->second.versions.find(it->second.current_version);
    if (ver_it == it->second.versions.end()) return std::nullopt;
    
    return ver_it->second;
}

std::vector<ModelVersion> ModelManager::getAllVersions(InferenceType type) const {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    
    auto it = pimpl_->deployments_.find(type);
    if (it == pimpl_->deployments_.end()) return {};
    
    std::vector<ModelVersion> versions;
    versions.reserve(it->second.versions.size());
    for (const auto& [v, ver] : it->second.versions) {
        versions.push_back(ver);
    }
    
    // Sort by upload time (newest first)
    std::sort(versions.begin(), versions.end(), 
              [](const ModelVersion& a, const ModelVersion& b) {
                  return a.uploaded_time_us > b.uploaded_time_us;
              });
    
    return versions;
}

ModelManager::DeploymentResult ModelManager::stageVersion(InferenceType type, const std::string& version) {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    DeploymentResult result = pimpl_->doStageVersion(type, version);
    
    if (deployment_callback_) {
        deployment_callback_(type, result);
    }
    
    return result;
}

ModelManager::DeploymentResult ModelManager::activateStagedVersion(InferenceType type) {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    DeploymentResult result = pimpl_->doActivateVersion(type);
    
    if (deployment_callback_) {
        deployment_callback_(type, result);
    }
    
    return result;
}

ModelManager::DeploymentResult ModelManager::rollbackVersion(InferenceType type) {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    DeploymentResult result = pimpl_->doRollback(type);
    
    if (deployment_callback_) {
        deployment_callback_(type, result);
    }
    
    return result;
}

ModelManager::DeploymentResult ModelManager::downloadAndStage(InferenceType type, 
                                                               const std::string& version,
                                                               const std::string& remote_url) {
    DeploymentResult result;
    
    // Download to staging directory
    std::string local_path = config_.staging_dir + "/" + 
        inferenceTypeToString(type) + "_" + version + ".onnx";
    
    if (progress_callback_) {
        progress_callback_(type, "downloading", 0.0f);
    }
    
    auto progress_cb = [this, type](float progress) {
        if (progress_callback_) {
            progress_callback_(type, "downloading", progress);
        }
    };
    
    size_t downloaded = pimpl_->downloadFile(remote_url, local_path, progress_cb);
    if (downloaded == 0) {
        result.error_message = "Download failed";
        return result;
    }
    
    // Verify checksum if provided
    if (config_.verify_checksum) {
        // Would need expected checksum from server
    }
    
    // Verify signature if required
    if (config_.require_signature && !config_.public_key_path.empty()) {
        std::string sig_path = local_path + ".sig";
        if (std::filesystem::exists(sig_path)) {
            if (!pimpl_->verifySignature(local_path, sig_path, config_.public_key_path)) {
                result.error_message = "Signature verification failed";
                std::filesystem::remove(local_path);
                return result;
            }
        }
    }
    
    // Register version
    ModelVersion ver;
    ver.version = version;
    ver.model_path = local_path;
    ver.checksum = pimpl_->computeSHA256(local_path);
    ver.size_bytes = downloaded;
    ver.uploaded_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    ver.runtime = "onnxruntime";  // Default
    ver.precision = "fp32";
    
    if (!registerModelVersion(type, ver)) {
        result.error_message = "Failed to register downloaded version";
        std::filesystem::remove(local_path);
        return result;
    }
    
    // Stage it
    result = stageVersion(type, version);
    
    if (progress_callback_) {
        progress_callback_(type, "complete", 1.0f);
    }
    
    return result;
}

bool ModelManager::verifyModel(const std::string& model_path, const std::string& expected_checksum) {
    std::string actual = pimpl_->computeSHA256(model_path);
    return actual == expected_checksum;
}

ModelDeployment ModelManager::getDeploymentStatus(InferenceType type) const {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    
    auto it = pimpl_->deployments_.find(type);
    if (it == pimpl_->deployments_.end()) {
        return ModelDeployment{};
    }
    return it->second;
}

std::vector<ModelDeployment> ModelManager::listDeployments() const {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    
    std::vector<ModelDeployment> deployments;
    deployments.reserve(pimpl_->deployments_.size());
    for (const auto& [type, dep] : pimpl_->deployments_) {
        deployments.push_back(dep);
    }
    return deployments;
}

int ModelManager::cleanupOldVersions(InferenceType type, int keep_count) {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    
    if (keep_count < 0) keep_count = config_.max_versions_to_keep;
    
    auto it = pimpl_->deployments_.find(type);
    if (it == pimpl_->deployments_.end()) return 0;
    
    auto& deployment = it->second;
    if (deployment.versions.size() <= keep_count) return 0;
    
    // Get versions sorted by upload time
    std::vector<std::pair<std::string, ModelVersion>> versions;
    for (const auto& [v, ver] : deployment.versions) {
        versions.emplace_back(v, ver);
    }
    
    std::sort(versions.begin(), versions.end(),
              [](const auto& a, const auto& b) {
                  return a.second.uploaded_time_us > b.second.uploaded_time_us;
              });
    
    int removed = 0;
    for (size_t i = keep_count; i < versions.size(); ++i) {
        const auto& [v, ver] = versions[i];
        
        // Don't remove active or staged
        if (ver.is_active || ver.is_staged) continue;
        
        // Remove file
        if (std::filesystem::exists(ver.model_path)) {
            std::filesystem::remove(ver.model_path);
        }
        
        deployment.versions.erase(v);
        removed++;
    }
    
    return removed;
}

std::string ModelManager::exportRegistry() const {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    
    json j;
    for (const auto& [type, dep] : pimpl_->deployments_) {
        json dep_json;
        dep_json["type"] = static_cast<int>(type);
        dep_json["current_version"] = dep.current_version;
        dep_json["staged_version"] = dep.staged_version;
        dep_json["active_runtime"] = dep.active_runtime;
        dep_json["active_precision"] = dep.active_precision;
        
        json versions_json;
        for (const auto& [v, ver] : dep.versions) {
            json ver_json;
            ver_json["version"] = ver.version;
            ver_json["model_path"] = ver.model_path;
            ver_json["checksum"] = ver.checksum;
            ver_json["uploaded_time_us"] = ver.uploaded_time_us;
            ver_json["activated_time_us"] = ver.activated_time_us;
            ver_json["is_active"] = ver.is_active;
            ver_json["is_staged"] = ver.is_staged;
            ver_json["runtime"] = ver.runtime;
            ver_json["precision"] = ver.precision;
            ver_json["size_bytes"] = ver.size_bytes;
            ver_json["metadata"] = ver.metadata;
            versions_json[v] = ver_json;
        }
        dep_json["versions"] = versions_json;
        j[std::to_string(static_cast<int>(type))] = dep_json;
    }
    
    return j.dump(2);
}

bool ModelManager::importRegistry(const std::string& json_str) {
    std::lock_guard<std::mutex> lock(pimpl_->mutex_);
    
    try {
        json j = json::parse(json_str);
        
        for (const auto& [type_str, dep_json] : j.items()) {
            InferenceType type = static_cast<InferenceType>(std::stoi(type_str));
            
            ModelDeployment dep;
            dep.type = type;
            dep.current_version = dep_json.value("current_version", "");
            dep.staged_version = dep_json.value("staged_version", "");
            dep.active_runtime = dep_json.value("active_runtime", "");
            dep.active_precision = dep_json.value("active_precision", "");
            
            for (const auto& [v, ver_json] : dep_json["versions"].items()) {
                ModelVersion ver;
                ver.version = ver_json.value("version", v);
                ver.model_path = ver_json.value("model_path", "");
                ver.checksum = ver_json.value("checksum", "");
                ver.uploaded_time_us = ver_json.value("uploaded_time_us", 0);
                ver.activated_time_us = ver_json.value("activated_time_us", 0);
                ver.is_active = ver_json.value("is_active", false);
                ver.is_staged = ver_json.value("is_staged", false);
                ver.runtime = ver_json.value("runtime", "onnxruntime");
                ver.precision = ver_json.value("precision", "fp32");
                ver.size_bytes = ver_json.value("size_bytes", 0);
                ver.metadata = ver_json.value("metadata", std::map<std::string, std::string>{});
                
                dep.versions[ver.version] = ver;
            }
            
            pimpl_->deployments_[type] = dep;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to import registry: " << e.what() << std::endl;
        return false;
    }
}

}  // namespace swarm::ai