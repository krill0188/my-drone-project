#include "log_analyzer.hpp"
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>

namespace swarm::interface {

using json = nlohmann::json;

LogAnalyzer::LogAnalyzer() = default;
LogAnalyzer::~LogAnalyzer() = default;

bool LogAnalyzer::initialize(const LogAnalyzerConfig& config) {
    _config = config;
    std::filesystem::create_directories(_config.output_directory);
    _initialized = true;
    return true;
}

std::vector<LogAnalyzer::MessageRecord> LogAnalyzer::readMessages(const std::string& bag_path) {
    std::vector<MessageRecord> messages;
    
    try {
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        reader.open(storage_options, converter_options);
        
        while (reader.has_next()) {
            auto msg = reader.read_next();
            if (!msg) continue;
            
            MessageRecord record;
            record.topic_name = msg->topic_name;
            record.timestamp_us = msg->time_stamp / 1000;
            
            if (msg->serialized_data && msg->serialized_data->buffer) {
                record.data.assign(
                    msg->serialized_data->buffer,
                    msg->serialized_data->buffer + msg->serialized_data->buffer_length
                );
            }
            
            messages.push_back(std::move(record));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error reading bag " << bag_path << ": " << e.what() << std::endl;
    }
    
    return messages;
}

AnalysisResult LogAnalyzer::analyzeLog(const std::string& log_path) {
    AnalysisResult result;
    result.log_path = log_path;
    
    if (!_initialized) {
        result.error_message = "LogAnalyzer not initialized";
        return result;
    }
    
    if (!std::filesystem::exists(log_path)) {
        result.error_message = "Log file not found: " + log_path;
        return result;
    }
    
    try {
        // Read messages
        auto messages = readMessages(log_path);
        
        if (messages.empty()) {
            result.error_message = "Log file is empty";
            return result;
        }
        
        // Parse messages for analysis
        for (auto& msg : messages) {
            if (msg.topic_name.find("vehicle_local_position") != std::string::npos) {
                parseVehicleLocalPosition(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("vehicle_attitude") != std::string::npos) {
                parseVehicleAttitude(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("battery_status") != std::string::npos) {
                parseBatteryStatus(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("vehicle_gps_position") != std::string::npos) {
                parseVehicleGpsPosition(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("estimator_status") != std::string::npos) {
                parseEstimatorStatus(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("actuator_outputs") != std::string::npos) {
                parseActuatorOutputs(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("comm_status") != std::string::npos) {
                parseCommStatus(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("ai_inference") != std::string::npos) {
                parseAIInference(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("safety_events") != std::string::npos) {
                parseSafetyEvent(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("formation_state") != std::string::npos) {
                parseFormationState(msg.data, msg.parsed_fields);
            } else if (msg.topic_name.find("task_allocation") != std::string::npos) {
                parseTaskAllocation(msg.data, msg.parsed_fields);
            }
        }
        
        // Analyze flight metrics
        result.metrics = analyzeFlightMetrics(messages);
        
        // Detect anomalies
        if (_config.detect_anomalies) {
            result.anomalies = detectAnomalies(messages);
        }
        
        // Extract time series data
        std::vector<std::string> key_topics = {
            "vehicle_local_position", "vehicle_attitude", "battery_status",
            "vehicle_gps_position", "estimator_status", "actuator_outputs",
            "comm_status", "ai_inference"
        };
        
        for (const auto& topic : key_topics) {
            auto it = std::find_if(messages.begin(), messages.end(),
                [&topic](const MessageRecord& m) { return m.topic_name.find(topic) != std::string::npos; });
            if (it != messages.end()) {
                // Extract common fields
                for (const auto& [field, _] : it->parsed_fields) {
                    auto ts = extractTimeSeries(messages, topic, field);
                    if (!ts.empty()) {
                        std::vector<double> values;
                        values.reserve(ts.size());
                        for (const auto& [t, v] : ts) values.push_back(v);
                        result.time_series_data[topic + "." + field] = values;
                    }
                }
            }
        }
        
        // Generate reports
        std::string base_name = std::filesystem::path(log_path).stem().string();
        std::string output_dir = _config.output_directory + "/" + base_name + "_analysis";
        std::filesystem::create_directories(output_dir);
        
        if (_config.generate_csv) {
            result.csv_report_path = output_dir + "/metrics.csv";
            generateCSVReport(result, result.csv_report_path);
        }
        
        if (_config.generate_json) {
            result.json_report_path = output_dir + "/metrics.json";
            generateJSONReport(result, result.json_report_path);
        }
        
        result.summary_report_path = output_dir + "/summary.txt";
        generateSummaryReport(result, result.summary_report_path);
        
        result.success = true;
        
    } catch (const std::exception& e) {
        result.error_message = "Exception during analysis: " + std::string(e.what());
    }
    
    return result;
}

std::vector<AnalysisResult> LogAnalyzer::analyzeSwarmLogs(const std::vector<std::string>& log_paths) {
    std::vector<AnalysisResult> results;
    results.reserve(log_paths.size());
    
    for (const auto& path : log_paths) {
        results.push_back(analyzeLog(path));
    }
    
    return results;
}

LogAnalyzer::FlightMetrics LogAnalyzer::analyzeFlightMetrics(
    const std::vector<MessageRecord>& messages) {
    
    FlightMetrics metrics;
    
    // Find flight start/end
    uint64_t first_ts = UINT64_MAX;
    uint64_t last_ts = 0;
    for (const auto& msg : messages) {
        first_ts = std::min(first_ts, msg.timestamp_us);
        last_ts = std::max(last_ts, msg.timestamp_us);
    }
    metrics.flight_start_us = first_ts;
    metrics.flight_end_us = last_ts;
    metrics.flight_duration_s = (last_ts - first_ts) / 1e6;
    
    // Position analysis
    std::vector<double> altitudes;
    std::vector<double> velocities;
    std::vector<double> positions_x, positions_y, positions_z;
    double prev_x = 0, prev_y = 0, prev_z = 0;
    bool first_pos = true;
    
    for (const auto& msg : messages) {
        if (msg.topic_name.find("vehicle_local_position") != std::string::npos) {
            auto it_z = msg.parsed_fields.find("z");
            auto it_vx = msg.parsed_fields.find("vx");
            auto it_vy = msg.parsed_fields.find("vy");
            auto it_vz = msg.parsed_fields.find("vz");
            auto it_x = msg.parsed_fields.find("x");
            auto it_y = msg.parsed_fields.find("y");
            
            if (it_z != msg.parsed_fields.end()) {
                double alt = -it_z->second;  // NED to altitude
                altitudes.push_back(alt);
                metrics.max_altitude_m = std::max(metrics.max_altitude_m, alt);
                metrics.min_altitude_m = std::min(metrics.min_altitude_m, alt);
            }
            
            if (it_vx != msg.parsed_fields.end() && 
                it_vy != msg.parsed_fields.end() && 
                it_vz != msg.parsed_fields.end()) {
                double vel = std::sqrt(
                    it_vx->second * it_vx->second +
                    it_vy->second * it_vy->second +
                    it_vz->second * it_vz->second
                );
                velocities.push_back(vel);
                metrics.max_velocity_mps = std::max(metrics.max_velocity_mps, vel);
            }
            
            if (it_x != msg.parsed_fields.end() && 
                it_y != msg.parsed_fields.end() && 
                it_z != msg.parsed_fields.end()) {
                if (!first_pos) {
                    double dx = it_x->second - prev_x;
                    double dy = it_y->second - prev_y;
                    double dz = it_z->second - prev_z;
                    metrics.total_distance_m += std::sqrt(dx*dx + dy*dy + dz*dz);
                }
                prev_x = it_x->second;
                prev_y = it_y->second;
                prev_z = it_z->second;
                first_pos = false;
            }
        }
    }
    
    if (!altitudes.empty()) {
        metrics.avg_altitude_m = std::accumulate(altitudes.begin(), altitudes.end(), 0.0) / altitudes.size();
    }
    if (!velocities.empty()) {
        metrics.avg_velocity_mps = std::accumulate(velocities.begin(), velocities.end(), 0.0) / velocities.size();
    }
    
    // Attitude analysis
    std::vector<double> rolls, pitches, yaw_rates;
    for (const auto& msg : messages) {
        if (msg.topic_name.find("vehicle_attitude") != std::string::npos) {
            auto it_roll = msg.parsed_fields.find("roll");
            auto it_pitch = msg.parsed_fields.find("pitch");
            auto it_yaw_rate = msg.parsed_fields.find("yaw_rate");
            
            if (it_roll != msg.parsed_fields.end()) {
                double roll_deg = it_roll->second * 180.0 / M_PI;
                rolls.push_back(std::abs(roll_deg));
                metrics.max_roll_deg = std::max(metrics.max_roll_deg, std::abs(roll_deg));
            }
            if (it_pitch != msg.parsed_fields.end()) {
                double pitch_deg = it_pitch->second * 180.0 / M_PI;
                pitches.push_back(std::abs(pitch_deg));
                metrics.max_pitch_deg = std::max(metrics.max_pitch_deg, std::abs(pitch_deg));
            }
            if (it_yaw_rate != msg.parsed_fields.end()) {
                double yaw_rate_dps = it_yaw_rate->second * 180.0 / M_PI;
                yaw_rates.push_back(std::abs(yaw_rate_dps));
                metrics.max_yaw_rate_dps = std::max(metrics.max_yaw_rate_dps, std::abs(yaw_rate_dps));
            }
        }
    }
    
    // Battery analysis
    std::vector<double> voltages, currents;
    double prev_current = 0;
    bool first_batt = true;
    for (const auto& msg : messages) {
        if (msg.topic_name.find("battery_status") != std::string::npos) {
            auto it_v = msg.parsed_fields.find("voltage");
            auto it_i = msg.parsed_fields.find("current");
            
            if (it_v != msg.parsed_fields.end()) {
                voltages.push_back(it_v->second);
                if (metrics.min_battery_voltage_v == 0 || it_v->second < metrics.min_battery_voltage_v) {
                    metrics.min_battery_voltage_v = it_v->second;
                }
            }
            if (it_i != msg.parsed_fields.end()) {
                currents.push_back(std::abs(it_i->second));
                metrics.max_battery_current_a = std::max(metrics.max_battery_current_a, std::abs(it_i->second));
                
                if (!first_batt) {
                    // Integrate current over time (approximate)
                    metrics.battery_consumed_mah += std::abs(it_i->second) * 0.1 / 3600.0 * 1000;  // 0.1s interval
                }
                first_batt = false;
            }
        }
    }
    if (!currents.empty()) {
        metrics.avg_battery_current_a = std::accumulate(currents.begin(), currents.end(), 0.0) / currents.size();
    }
    
    // GPS analysis
    std::vector<double> hdops;
    std::vector<int> satellites;
    for (const auto& msg : messages) {
        if (msg.topic_name.find("vehicle_gps_position") != std::string::npos) {
            auto it_hdop = msg.parsed_fields.find("hdop");
            auto it_sats = msg.parsed_fields.find("satellites");
            auto it_fix = msg.parsed_fields.find("fix_type");
            
            if (it_hdop != msg.parsed_fields.end()) {
                hdops.push_back(it_hdop->second);
                metrics.max_gps_hdop = std::max(metrics.max_gps_hdop, it_hdop->second);
            }
            if (it_sats != msg.parsed_fields.end()) {
                satellites.push_back(static_cast<int>(it_sats->second));
            }
            if (it_fix != msg.parsed_fields.end() && it_fix->second < 3) {
                metrics.gps_fix_lost_count++;
            }
        }
    }
    if (!hdops.empty()) {
        metrics.avg_gps_hdop = std::accumulate(hdops.begin(), hdops.end(), 0.0) / hdops.size();
    }
    if (!satellites.empty()) {
        metrics.gps_satellites_avg = static_cast<int>(
            std::accumulate(satellites.begin(), satellites.end(), 0) / satellites.size());
    }
    
    // Estimator analysis
    std::vector<double> innovations;
    for (const auto& msg : messages) {
        if (msg.topic_name.find("estimator_status") != std::string::npos) {
            auto it_innov = msg.parsed_fields.find("innovation");
            auto it_reset = msg.parsed_fields.find("reset_count");
            
            if (it_innov != msg.parsed_fields.end()) {
                innovations.push_back(it_innov->second);
                metrics.max_estimator_innovation = std::max(metrics.max_estimator_innovation, it_innov->second);
            }
            if (it_reset != msg.parsed_fields.end()) {
                metrics.estimator_reset_count = static_cast<int>(it_reset->second);
            }
        }
    }
    if (!innovations.empty()) {
        metrics.avg_estimator_innovation = std::accumulate(innovations.begin(), innovations.end(), 0.0) / innovations.size();
    }
    
    // Actuator analysis
    std::vector<double> actuator_outputs;
    for (const auto& msg : messages) {
        if (msg.topic_name.find("actuator_outputs") != std::string::npos) {
            for (int i = 0; i < 8; ++i) {
                std::string field = "output" + std::to_string(i);
                auto it = msg.parsed_fields.find(field);
                if (it != msg.parsed_fields.end()) {
                    actuator_outputs.push_back(std::abs(it->second));
                    metrics.max_actuator_output = std::max(metrics.max_actuator_output, std::abs(it->second));
                    if (std::abs(it->second) > 0.95) {
                        metrics.actuator_saturation_count++;
                    }
                }
            }
        }
    }
    if (!actuator_outputs.empty()) {
        metrics.avg_actuator_output = std::accumulate(actuator_outputs.begin(), actuator_outputs.end(), 0.0) / actuator_outputs.size();
    }
    
    // Communication analysis
    std::vector<double> link_qualities, latencies;
    for (const auto& msg : messages) {
        if (msg.topic_name.find("comm_status") != std::string::npos) {
            auto it_lq = msg.parsed_fields.find("link_quality");
            auto it_loss = msg.parsed_fields.find("packet_loss");
            auto it_lat = msg.parsed_fields.find("latency_ms");
            
            if (it_lq != msg.parsed_fields.end()) {
                link_qualities.push_back(it_lq->second);
                metrics.min_link_quality = std::min(metrics.min_link_quality, it_lq->second);
            }
            if (it_loss != msg.parsed_fields.end() && it_loss->second > 0) {
                metrics.packet_loss_count++;
            }
            if (it_lat != msg.parsed_fields.end()) {
                latencies.push_back(it_lat->second);
                metrics.max_latency_ms = std::max(metrics.max_latency_ms, it_lat->second);
            }
        }
    }
    if (!link_qualities.empty()) {
        metrics.avg_link_quality = std::accumulate(link_qualities.begin(), link_qualities.end(), 0.0) / link_qualities.size();
    }
    if (!latencies.empty()) {
        metrics.avg_latency_ms = std::accumulate(latencies.begin(), latencies.end(), 0.0) / latencies.size();
    }
    
    // AI Inference analysis
    for (const auto& msg : messages) {
        if (msg.topic_name.find("ai_inference") != std::string::npos) {
            metrics.total_inferences++;
            
            auto it_time = msg.parsed_fields.find("inference_time_ms");
            if (it_time != msg.parsed_fields.end()) {
                metrics.avg_inference_time_ms = 
                    (metrics.avg_inference_time_ms * (metrics.total_inferences - 1) + it_time->second) / metrics.total_inferences;
                metrics.max_inference_time_ms = std::max(metrics.max_inference_time_ms, it_time->second);
            }
            
            auto it_class = msg.parsed_fields.find("class_id");
            auto it_name = msg.parsed_fields.find("class_name");
            if (it_class != msg.parsed_fields.end() && it_name != msg.parsed_fields.end()) {
                std::string key = std::to_string(static_cast<int>(it_class->second)) + "_" + 
                                  std::to_string(static_cast<int>(it_name->second));
                metrics.detections_by_class[key]++;
            }
        }
    }
    
    // Safety Events analysis
    for (const auto& msg : messages) {
        if (msg.topic_name.find("safety_events") != std::string::npos) {
            metrics.safety_events_count++;
            
            auto it_type = msg.parsed_fields.find("event_type");
            auto it_sev = msg.parsed_fields.find("severity");
            
            if (it_type != msg.parsed_fields.end()) {
                std::string type_str = std::to_string(static_cast<int>(it_type->second));
                metrics.safety_events_by_type[type_str]++;
                
                if (type_str == "0" || type_str == "GEOFENCE") metrics.geofence_breaches++;
                if (type_str == "2" || type_str == "ESTOP") metrics.emergency_stops++;
            }
        }
    }
    
    // Formation analysis
    std::vector<double> formation_errors;
    for (const auto& msg : messages) {
        if (msg.topic_name.find("formation_state") != std::string::npos) {
            auto it_error = msg.parsed_fields.find("formation_error");
            auto it_loss = msg.parsed_fields.find("formation_lost");
            
            if (it_error != msg.parsed_fields.end()) {
                formation_errors.push_back(it_error->second);
                metrics.max_formation_error_m = std::max(metrics.max_formation_error_m, it_error->second);
            }
            if (it_loss != msg.parsed_fields.end() && it_loss->second > 0) {
                metrics.formation_loss_count++;
            }
        }
    }
    if (!formation_errors.empty()) {
        metrics.avg_formation_error_m = std::accumulate(formation_errors.begin(), formation_errors.end(), 0.0) / formation_errors.size();
    }
    
    // Task Allocation analysis
    std::vector<double> alloc_times;
    for (const auto& msg : messages) {
        if (msg.topic_name.find("task_allocation") != std::string::npos) {
            auto it_status = msg.parsed_fields.find("status");
            auto it_time = msg.parsed_fields.find("allocation_time_ms");
            
            if (it_status != msg.parsed_fields.end()) {
                if (it_status->second == 2) metrics.tasks_completed++;
                else if (it_status->second == 3) metrics.tasks_failed++;
                else metrics.tasks_allocated++;
            }
            if (it_time != msg.parsed_fields.end()) {
                alloc_times.push_back(it_time->second);
            }
        }
    }
    if (!alloc_times.empty()) {
        metrics.avg_allocation_time_ms = std::accumulate(alloc_times.begin(), alloc_times.end(), 0.0) / alloc_times.size();
    }
    
    return metrics;
}

std::vector<LogAnalyzer::AnomalyEvent> LogAnalyzer::detectAnomalies(
    const std::vector<MessageRecord>& messages) {
    
    std::vector<AnomalyEvent> anomalies;
    
    // Collect statistics per topic.field
    std::map<std::string, std::vector<double>> field_values;
    
    for (const auto& msg : messages) {
        for (const auto& [field, value] : msg.parsed_fields) {
            std::string key = msg.topic_name + "." + field;
            field_values[key].push_back(value);
        }
    }
    
    // Compute statistics for each field
    std::map<std::string, Statistics> field_stats;
    for (auto& [key, values] : field_values) {
        field_stats[key] = computeStatistics(values);
    }
    
    // Check each message for anomalies
    for (const auto& msg : messages) {
        for (const auto& [field, value] : msg.parsed_fields) {
            std::string key = msg.topic_name + "." + field;
            auto it = field_stats.find(key);
            if (it != field_stats.end()) {
                checkValueAnomaly(msg.topic_name, field, value, msg.timestamp_us, it->second, anomalies);
            }
        }
    }
    
    return anomalies;
}

void LogAnalyzer::checkValueAnomaly(const std::string& topic, const std::string& field,
                                    double value, uint64_t timestamp,
                                    const Statistics& stats,
                                    std::vector<AnomalyEvent>& anomalies) {
    
    double threshold = _config.anomaly_threshold_std * stats.std_dev;
    double expected_min = stats.mean - threshold;
    double expected_max = stats.mean + threshold;
    
    if (value < expected_min || value > expected_max) {
        AnomalyEvent anomaly;
        anomaly.timestamp_us = timestamp;
        anomaly.topic = topic;
        anomaly.field = field;
        anomaly.value = value;
        anomaly.expected_min = expected_min;
        anomaly.expected_max = expected_max;
        anomaly.severity = std::min(1.0, std::abs(value - stats.mean) / (3.0 * stats.std_dev));
        anomaly.description = field + " value " + std::to_string(value) + 
                              " outside expected range [" + std::to_string(expected_min) + 
                              ", " + std::to_string(expected_max) + "]";
        anomalies.push_back(anomaly);
    }
}

LogAnalyzer::Statistics LogAnalyzer::computeStatistics(const std::vector<double>& values) {
    Statistics stats;
    if (values.empty()) return stats;
    
    stats.count = values.size();
    stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    
    double variance = 0.0;
    for (double v : values) {
        variance += (v - stats.mean) * (v - stats.mean);
    }
    stats.std_dev = std::sqrt(variance / values.size());
    
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    
    stats.min = sorted.front();
    stats.max = sorted.back();
    stats.median = sorted[sorted.size() / 2];
    stats.q1 = sorted[sorted.size() / 4];
    stats.q3 = sorted[3 * sorted.size() / 4];
    
    return stats;
}

std::vector<std::pair<uint64_t, double>> LogAnalyzer::extractTimeSeries(
    const std::vector<MessageRecord>& messages,
    const std::string& topic,
    const std::string& field) {
    
    std::vector<std::pair<uint64_t, double>> result;
    
    for (const auto& msg : messages) {
        if (msg.topic_name.find(topic) != std::string::npos) {
            auto it = msg.parsed_fields.find(field);
            if (it != msg.parsed_fields.end()) {
                result.emplace_back(msg.timestamp_us, it->second);
            }
        }
    }
    
    return result;
}

void LogAnalyzer::generateCSVReport(const AnalysisResult& result, 
                                    const std::string& output_path) const {
    std::ofstream csv(output_path);
    if (!csv.is_open()) return;
    
    // Header
    csv << "Metric,Value,Unit\n";
    
    // Flight metrics
    csv << "Flight Duration," << result.metrics.flight_duration_s << ",s\n";
    csv << "Total Distance," << result.metrics.total_distance_m << ",m\n";
    csv << "Max Altitude," << result.metrics.max_altitude_m << ",m\n";
    csv << "Min Altitude," << result.metrics.min_altitude_m << ",m\n";
    csv << "Avg Altitude," << result.metrics.avg_altitude_m << ",m\n";
    csv << "Max Velocity," << result.metrics.max_velocity_mps << ",m/s\n";
    csv << "Avg Velocity," << result.metrics.avg_velocity_mps << ",m/s\n";
    csv << "Max Roll," << result.metrics.max_roll_deg << ",deg\n";
    csv << "Max Pitch," << result.metrics.max_pitch_deg << ",deg\n";
    csv << "Max Yaw Rate," << result.metrics.max_yaw_rate_dps << ",deg/s\n";
    csv << "Min Battery Voltage," << result.metrics.min_battery_voltage_v << ",V\n";
    csv << "Max Battery Current," << result.metrics.max_battery_current_a << ",A\n";
    csv << "Avg Battery Current," << result.metrics.avg_battery_current_a << ",A\n";
    csv << "Battery Consumed," << result.metrics.battery_consumed_mah << ",mAh\n";
    csv << "Avg GPS HDOP," << result.metrics.avg_gps_hdop << ",\n";
    csv << "Max GPS HDOP," << result.metrics.max_gps_hdop << ",\n";
    csv << "GPS Satellites Avg," << result.metrics.gps_satellites_avg << ",\n";
    csv << "GPS Fix Lost Count," << result.metrics.gps_fix_lost_count << ",\n";
    csv << "Avg Estimator Innovation," << result.metrics.avg_estimator_innovation << ",\n";
    csv << "Max Estimator Innovation," << result.metrics.max_estimator_innovation << ",\n";
    csv << "Estimator Reset Count," << result.metrics.estimator_reset_count << ",\n";
    csv << "Max Actuator Output," << result.metrics.max_actuator_output << ",\n";
    csv << "Avg Actuator Output," << result.metrics.avg_actuator_output << ",\n";
    csv << "Actuator Saturation Count," << result.metrics.actuator_saturation_count << ",\n";
    csv << "Avg Link Quality," << result.metrics.avg_link_quality << ",\n";
    csv << "Min Link Quality," << result.metrics.min_link_quality << ",\n";
    csv << "Packet Loss Count," << result.metrics.packet_loss_count << ",\n";
    csv << "Avg Latency," << result.metrics.avg_latency_ms << ",ms\n";
    csv << "Max Latency," << result.metrics.max_latency_ms << ",ms\n";
    csv << "Total Inferences," << result.metrics.total_inferences << ",\n";
    csv << "Avg Inference Time," << result.metrics.avg_inference_time_ms << ",ms\n";
    csv << "Max Inference Time," << result.metrics.max_inference_time_ms << ",ms\n";
    csv << "Safety Events," << result.metrics.safety_events_count << ",\n";
    csv << "Geofence Breaches," << result.metrics.geofence_breaches << ",\n";
    csv << "Emergency Stops," << result.metrics.emergency_stops << ",\n";
    csv << "Avg Formation Error," << result.metrics.avg_formation_error_m << ",m\n";
    csv << "Max Formation Error," << result.metrics.max_formation_error_m << ",m\n";
    csv << "Formation Loss Count," << result.metrics.formation_loss_count << ",\n";
    csv << "Tasks Allocated," << result.metrics.tasks_allocated << ",\n";
    csv << "Tasks Completed," << result.metrics.tasks_completed << ",\n";
    csv << "Tasks Failed," << result.metrics.tasks_failed << ",\n";
    csv << "Avg Allocation Time," << result.metrics.avg_allocation_time_ms << ",ms\n";
    
    // Anomalies
    csv << "\nAnomalies\n";
    csv << "Timestamp (us),Topic,Field,Value,Expected Min,Expected Max,Severity,Description\n";
    for (const auto& anomaly : result.anomalies) {
        csv << anomaly.timestamp_us << "," << anomaly.topic << "," << anomaly.field << ","
            << anomaly.value << "," << anomaly.expected_min << "," << anomaly.expected_max << ","
            << anomaly.severity << "," << anomaly.description << "\n";
    }
}

void LogAnalyzer::generateJSONReport(const AnalysisResult& result, 
                                     const std::string& output_path) const {
    json j;
    
    j["log_path"] = result.log_path;
    j["success"] = result.success;
    
    // Metrics
    json metrics;
    metrics["flight_duration_s"] = result.metrics.flight_duration_s;
    metrics["total_distance_m"] = result.metrics.total_distance_m;
    metrics["max_altitude_m"] = result.metrics.max_altitude_m;
    metrics["min_altitude_m"] = result.metrics.min_altitude_m;
    metrics["avg_altitude_m"] = result.metrics.avg_altitude_m;
    metrics["max_velocity_mps"] = result.metrics.max_velocity_mps;
    metrics["avg_velocity_mps"] = result.metrics.avg_velocity_mps;
    metrics["max_roll_deg"] = result.metrics.max_roll_deg;
    metrics["max_pitch_deg"] = result.metrics.max_pitch_deg;
    metrics["max_yaw_rate_dps"] = result.metrics.max_yaw_rate_dps;
    metrics["min_battery_voltage_v"] = result.metrics.min_battery_voltage_v;
    metrics["max_battery_current_a"] = result.metrics.max_battery_current_a;
    metrics["avg_battery_current_a"] = result.metrics.avg_battery_current_a;
    metrics["battery_consumed_mah"] = result.metrics.battery_consumed_mah;
    metrics["avg_gps_hdop"] = result.metrics.avg_gps_hdop;
    metrics["max_gps_hdop"] = result.metrics.max_gps_hdop;
    metrics["gps_satellites_avg"] = result.metrics.gps_satellites_avg;
    metrics["gps_fix_lost_count"] = result.metrics.gps_fix_lost_count;
    metrics["avg_estimator_innovation"] = result.metrics.avg_estimator_innovation;
    metrics["max_estimator_innovation"] = result.metrics.max_estimator_innovation;
    metrics["estimator_reset_count"] = result.metrics.estimator_reset_count;
    metrics["max_actuator_output"] = result.metrics.max_actuator_output;
    metrics["avg_actuator_output"] = result.metrics.avg_actuator_output;
    metrics["actuator_saturation_count"] = result.metrics.actuator_saturation_count;
    metrics["avg_link_quality"] = result.metrics.avg_link_quality;
    metrics["min_link_quality"] = result.metrics.min_link_quality;
    metrics["packet_loss_count"] = result.metrics.packet_loss_count;
    metrics["avg_latency_ms"] = result.metrics.avg_latency_ms;
    metrics["max_latency_ms"] = result.metrics.max_latency_ms;
    metrics["total_inferences"] = result.metrics.total_inferences;
    metrics["avg_inference_time_ms"] = result.metrics.avg_inference_time_ms;
    metrics["max_inference_time_ms"] = result.metrics.max_inference_time_ms;
    metrics["detections_by_class"] = result.metrics.detections_by_class;
    metrics["safety_events_count"] = result.metrics.safety_events_count;
    metrics["safety_events_by_type"] = result.metrics.safety_events_by_type;
    metrics["geofence_breaches"] = result.metrics.geofence_breaches;
    metrics["emergency_stops"] = result.metrics.emergency_stops;
    metrics["avg_formation_error_m"] = result.metrics.avg_formation_error_m;
    metrics["max_formation_error_m"] = result.metrics.max_formation_error_m;
    metrics["formation_loss_count"] = result.metrics.formation_loss_count;
    metrics["tasks_allocated"] = result.metrics.tasks_allocated;
    metrics["tasks_completed"] = result.metrics.tasks_completed;
    metrics["tasks_failed"] = result.metrics.tasks_failed;
    metrics["avg_allocation_time_ms"] = result.metrics.avg_allocation_time_ms;
    
    j["metrics"] = metrics;
    
    // Anomalies
    json anomalies_json = json::array();
    for (const auto& anomaly : result.anomalies) {
        json a;
        a["timestamp_us"] = anomaly.timestamp_us;
        a["topic"] = anomaly.topic;
        a["field"] = anomaly.field;
        a["value"] = anomaly.value;
        a["expected_min"] = anomaly.expected_min;
        a["expected_max"] = anomaly.expected_max;
        a["severity"] = anomaly.severity;
        a["description"] = anomaly.description;
        anomalies_json.push_back(a);
    }
    j["anomalies"] = anomalies_json;
    
    // Time series (sampled)
    json ts_json;
    for (const auto& [key, values] : result.time_series_data) {
        // Downsample to max 1000 points
        json sampled = json::array();
        size_t step = std::max<size_t>(1, values.size() / 1000);
        for (size_t i = 0; i < values.size(); i += step) {
            sampled.push_back(values[i]);
        }
        ts_json[key] = sampled;
    }
    j["time_series"] = ts_json;
    
    std::ofstream f(output_path);
    if (f.is_open()) {
        f << j.dump(2);
    }
}

void LogAnalyzer::generateSummaryReport(const AnalysisResult& result, 
                                        const std::string& output_path) const {
    std::ofstream f(output_path);
    if (!f.is_open()) return;
    
    f << "========================================\n";
    f << "FLIGHT LOG ANALYSIS SUMMARY\n";
    f << "========================================\n\n";
    
    f << "Log File: " << result.log_path << "\n";
    f << "Analysis Success: " << (result.success ? "YES" : "NO") << "\n\n";
    
    if (!result.success) {
        f << "Error: " << result.error_message << "\n";
        return;
    }
    
    f << "----------------------------------------\n";
    f << "FLIGHT OVERVIEW\n";
    f << "----------------------------------------\n";
    f << "Duration: " << std::fixed << std::setprecision(1) << result.metrics.flight_duration_s << " s\n";
    f << "Distance: " << std::fixed << std::setprecision(1) << result.metrics.total_distance_m << " m\n";
    f << "Altitude: " << std::fixed << std::setprecision(1) << result.metrics.min_altitude_m 
      << " - " << result.metrics.max_altitude_m << " m (avg: " << result.metrics.avg_altitude_m << " m)\n";
    f << "Velocity: " << std::fixed << std::setprecision(1) << result.metrics.avg_velocity_mps 
      << " m/s avg, " << result.metrics.max_velocity_mps << " m/s max\n\n";
    
    f << "----------------------------------------\n";
    f << "ATTITUDE\n";
    f << "----------------------------------------\n";
    f << "Max Roll: " << std::fixed << std::setprecision(1) << result.metrics.max_roll_deg << " deg\n";
    f << "Max Pitch: " << std::fixed << std::setprecision(1) << result.metrics.max_pitch_deg << " deg\n";
    f << "Max Yaw Rate: " << std::fixed << std::setprecision(1) << result.metrics.max_yaw_rate_dps << " deg/s\n\n";
    
    f << "----------------------------------------\n";
    f << "BATTERY\n";
    f << "----------------------------------------\n";
    f << "Min Voltage: " << std::fixed << std::setprecision(2) << result.metrics.min_battery_voltage_v << " V\n";
    f << "Current: " << std::fixed << std::setprecision(1) << result.metrics.avg_battery_current_a 
      << " A avg, " << result.metrics.max_battery_current_a << " A max\n";
    f << "Consumed: " << std::fixed << std::setprecision(0) << result.metrics.battery_consumed_mah << " mAh\n\n";
    
    f << "----------------------------------------\n";
    f << "GPS\n";
    f << "----------------------------------------\n";
    f << "HDOP: " << std::fixed << std::setprecision(1) << result.metrics.avg_gps_hdop 
      << " avg, " << result.metrics.max_gps_hdop << " max\n";
    f << "Satellites: " << result.metrics.gps_satellites_avg << " avg\n";
    f << "Fix Lost: " << result.metrics.gps_fix_lost_count << " times\n\n";
    
    f << "----------------------------------------\n";
    f << "ESTIMATOR\n";
    f << "----------------------------------------\n";
    f << "Innovation: " << std::fixed << std::setprecision(3) << result.metrics.avg_estimator_innovation 
      << " avg, " << result.metrics.max_estimator_innovation << " max\n";
    f << "Resets: " << result.metrics.estimator_reset_count << "\n\n";
    
    f << "----------------------------------------\n";
    f << "ACTUATORS\n";
    f << "----------------------------------------\n";
    f << "Output: " << std::fixed << std::setprecision(2) << result.metrics.avg_actuator_output 
      << " avg, " << result.metrics.max_actuator_output << " max\n";
    f << "Saturation Events: " << result.metrics.actuator_saturation_count << "\n\n";
    
    f << "----------------------------------------\n";
    f << "COMMUNICATION\n";
    f << "----------------------------------------\n";
    f << "Link Quality: " << std::fixed << std::setprecision(2) << result.metrics.avg_link_quality 
      << " avg, " << result.metrics.min_link_quality << " min\n";
    f << "Packet Loss: " << result.metrics.packet_loss_count << " events\n";
    f << "Latency: " << std::fixed << std::setprecision(1) << result.metrics.avg_latency_ms 
      << " ms avg, " << result.metrics.max_latency_ms << " ms max\n\n";
    
    f << "----------------------------------------\n";
    f << "AI INFERENCE\n";
    f << "----------------------------------------\n";
    f << "Total Inferences: " << result.metrics.total_inferences << "\n";
    f << "Inference Time: " << std::fixed << std::setprecision(1) << result.metrics.avg_inference_time_ms 
      << " ms avg, " << result.metrics.max_inference_time_ms << " ms max\n";
    f << "Detections:\n";
    for (const auto& [cls, count] : result.metrics.detections_by_class) {
        f << "  " << cls << ": " << count << "\n";
    }
    f << "\n";
    
    f << "----------------------------------------\n";
    f << "SAFETY EVENTS\n";
    f << "----------------------------------------\n";
    f << "Total: " << result.metrics.safety_events_count << "\n";
    f << "Geofence Breaches: " << result.metrics.geofence_breaches << "\n";
    f << "Emergency Stops: " << result.metrics.emergency_stops << "\n";
    for (const auto& [type, count] : result.metrics.safety_events_by_type) {
        f << "  Type " << type << ": " << count << "\n";
    }
    f << "\n";
    
    f << "----------------------------------------\n";
    f << "FORMATION\n";
    f << "----------------------------------------\n";
    f << "Error: " << std::fixed << std::setprecision(2) << result.metrics.avg_formation_error_m 
      << " m avg, " << result.metrics.max_formation_error_m << " m max\n";
    f << "Formation Loss: " << result.metrics.formation_loss_count << " events\n\n";
    
    f << "----------------------------------------\n";
    f << "TASK ALLOCATION\n";
    f << "----------------------------------------\n";
    f << "Allocated: " << result.metrics.tasks_allocated << "\n";
    f << "Completed: " << result.metrics.tasks_completed << "\n";
    f << "Failed: " << result.metrics.tasks_failed << "\n";
    f << "Avg Allocation Time: " << std::fixed << std::setprecision(1) 
      << result.metrics.avg_allocation_time_ms << " ms\n\n";
    
    f << "----------------------------------------\n";
    f << "ANOMALIES DETECTED: " << result.anomalies.size() << "\n";
    f << "----------------------------------------\n";
    for (const auto& anomaly : result.anomalies) {
        f << "[" << anomaly.timestamp_us << "] " << anomaly.topic << "." << anomaly.field
          << " = " << anomaly.value << " (expected: " << anomaly.expected_min 
          << " - " << anomaly.expected_max << ") severity: " << anomaly.severity << "\n";
    }
    
    f << "\n========================================\n";
    f << "END OF SUMMARY\n";
    f << "========================================\n";
}

// Message parsing implementations (simplified - would use actual message definitions)
void LogAnalyzer::parseVehicleLocalPosition(const std::vector<uint8_t>& data, 
                                            std::map<std::string, double>& fields) {
    // Simplified parsing - in reality would use CDR deserialization
    if (data.size() >= 48) {  // x, y, z, vx, vy, vz (6 * double = 48 bytes)
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["x"] = ptr[0];
        fields["y"] = ptr[1];
        fields["z"] = ptr[2];
        fields["vx"] = ptr[3];
        fields["vy"] = ptr[4];
        fields["vz"] = ptr[5];
    }
}

void LogAnalyzer::parseVehicleAttitude(const std::vector<uint8_t>& data, 
                                       std::map<std::string, double>& fields) {
    if (data.size() >= 32) {  // q[4], roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["roll"] = ptr[4];
        fields["pitch"] = ptr[5];
        fields["yaw"] = ptr[6];
        fields["roll_rate"] = ptr[7];
        fields["pitch_rate"] = ptr[8];
        fields["yaw_rate"] = ptr[9];
    }
}

void LogAnalyzer::parseBatteryStatus(const std::vector<uint8_t>& data, 
                                     std::map<std::string, double>& fields) {
    if (data.size() >= 16) {
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["voltage"] = ptr[0];
        fields["current"] = ptr[1];
    }
}

void LogAnalyzer::parseVehicleGpsPosition(const std::vector<uint8_t>& data, 
                                          std::map<std::string, double>& fields) {
    if (data.size() >= 40) {
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["lat"] = ptr[0];
        fields["lon"] = ptr[1];
        fields["alt"] = ptr[2];
        fields["hdop"] = ptr[3];
        fields["vdop"] = ptr[4];
    }
}

void LogAnalyzer::parseEstimatorStatus(const std::vector<uint8_t>& data, 
                                       std::map<std::string, double>& fields) {
    if (data.size() >= 8) {
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["innovation"] = ptr[0];
        fields["reset_count"] = ptr[1];
    }
}

void LogAnalyzer::parseActuatorOutputs(const std::vector<uint8_t>& data, 
                                       std::map<std::string, double>& fields) {
    if (data.size() >= 64) {  // 8 outputs
        const double* ptr = reinterpret_cast<const double*>(data.data());
        for (int i = 0; i < 8; ++i) {
            fields["output" + std::to_string(i)] = ptr[i];
        }
    }
}

void LogAnalyzer::parseCommStatus(const std::vector<uint8_t>& data, 
                                  std::map<std::string, double>& fields) {
    if (data.size() >= 24) {
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["link_quality"] = ptr[0];
        fields["packet_loss"] = ptr[1];
        fields["latency_ms"] = ptr[2];
        fields["rssi"] = ptr[3];
    }
}

void LogAnalyzer::parseAIInference(const std::vector<uint8_t>& data, 
                                   std::map<std::string, double>& fields) {
    if (data.size() >= 32) {
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["class_id"] = ptr[0];
        fields["class_name"] = ptr[1];  // Would be string in reality
        fields["confidence"] = ptr[2];
        fields["inference_time_ms"] = ptr[3];
    }
}

void LogAnalyzer::parseSafetyEvent(const std::vector<uint8_t>& data, 
                                   std::map<std::string, double>& fields) {
    if (data.size() >= 16) {
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["event_type"] = ptr[0];
        fields["severity"] = ptr[1];
        fields["source_id"] = ptr[2];
    }
}

void LogAnalyzer::parseFormationState(const std::vector<uint8_t>& data, 
                                      std::map<std::string, double>& fields) {
    if (data.size() >= 16) {
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["formation_error"] = ptr[0];
        fields["formation_lost"] = ptr[1];
    }
}

void LogAnalyzer::parseTaskAllocation(const std::vector<uint8_t>& data, 
                                      std::map<std::string, double>& fields) {
    if (data.size() >= 24) {
        const double* ptr = reinterpret_cast<const double*>(data.data());
        fields["task_id"] = ptr[0];
        fields["status"] = ptr[1];
        fields["allocation_time_ms"] = ptr[2];
    }
}

LogAnalyzer::FlightPhase LogAnalyzer::detectFlightPhase(const MessageRecord& msg, 
                                                         const MessageRecord* prev_msg) {
    // Simplified flight phase detection
    if (msg.topic_name.find("vehicle_local_position") != std::string::npos) {
        auto it_z = msg.parsed_fields.find("z");
        auto it_vz = msg.parsed_fields.find("vz");
        
        if (it_z != msg.parsed_fields.end() && it_vz != msg.parsed_fields.end()) {
            double alt = -it_z->second;
            double vz = it_vz->second;
            
            if (alt < 1.0 && std::abs(vz) < 0.5) return FlightPhase::PREFLIGHT;
            if (vz < -1.0) return FlightPhase::CLIMB;
            if (std::abs(vz) < 0.5 && alt > 5.0) return FlightPhase::CRUISE;
            if (vz > 1.0) return FlightPhase::DESCENT;
            if (alt < 2.0 && vz > 0) return FlightPhase::LANDING;
        }
    }
    return FlightPhase::UNKNOWN;
}

}  // namespace swarm::interface