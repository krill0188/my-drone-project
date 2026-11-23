#ifndef SWARM_MONITOR_PLUGIN_H
#define SWARM_MONITOR_PLUGIN_H

#include <QGCPlugin.h>
#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>
#include <QGeoCoordinate>
#include <QDateTime>
#include <QTimer>

class MissionManager;
class VehicleManager;
class LinkManager;
class QGCApplication;
class QmlObjectListModel;

namespace swarm::gcs {

struct SwarmVehicleStatus {
    uint8_t id = 0;
    QString callsign;
    QString firmware_type;  // "PX4", "ArduPilot"
    
    // Position & attitude
    QGeoCoordinate coordinate;
    double altitude_amsl = 0.0;
    double altitude_rel = 0.0;
    double heading_deg = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double ground_speed_mps = 0.0;
    double vertical_speed_mps = 0.0;
    
    // State
    QString flight_mode;
    QString arming_state;  // "ARMED", "DISARMED"
    QString nav_state;     // "AUTO", "LOITER", "RTL", etc.
    bool is_flying = false;
    bool is_leader = false;
    uint8_t leader_id = 0;
    
    // Battery
    double battery_voltage_v = 0.0;
    double battery_current_a = 0.0;
    double battery_remaining_pct = 0.0;
    double battery_consumed_mah = 0.0;
    int battery_cell_count = 0;
    QVector<double> cell_voltages_v;
    
    // GPS
    int gps_satellites = 0;
    double gps_hdop = 0.0;
    int gps_fix_type = 0;
    bool gps_healthy = false;
    
    // Estimator
    double ekf_innovation = 0.0;
    bool ekf_healthy = true;
    int ekf_reset_count = 0;
    
    // Communication
    double link_quality = 1.0;
    double rssi_dbm = -50.0;
    double snr_db = 20.0;
    double latency_ms = 10.0;
    uint64_t last_heartbeat_us = 0;
    uint64_t messages_received = 0;
    uint64_t messages_lost = 0;
    
    // AI/Perception
    int active_detections = 0;
    double inference_time_ms = 0.0;
    QString last_detection_class;
    double last_detection_confidence = 0.0;
    
    // Safety
    bool geofence_ok = true;
    bool estop_active = false;
    QString last_safety_event;
    
    // Formation
    double formation_error_m = 0.0;
    QGeoCoordinate formation_target;
    bool in_formation = false;
    
    // Timestamps
    QDateTime last_update;
    QDateTime first_seen;
};

struct SwarmFormationStatus {
    uint8_t leader_id = 0;
    QString formation_type;  // "line", "vee", "diamond", "grid", "custom"
    QVector<uint8_t> member_ids;
    QMap<uint8_t, QGeoCoordinate> assigned_positions;
    QMap<uint8_t, QGeoCoordinate> actual_positions;
    QMap<uint8_t, double> position_errors_m;
    double avg_formation_error_m = 0.0;
    double max_formation_error_m = 0.0;
    bool formation_locked = true;
    QString formation_state;  // "forming", "holding", "transitioning", "lost"
    QDateTime last_update;
};

struct MeshQoSMetrics {
    uint8_t node_id = 0;
    QMap<uint8_t, double> link_quality_to;      // node_id -> quality (0-1)
    QMap<uint8_t, double> rssi_dbm_to;          // node_id -> RSSI
    QMap<uint8_t, double> latency_ms_to;        // node_id -> latency
    QMap<uint8_t, uint64_t> packets_sent_to;
    QMap<uint8_t, uint64_t> packets_received_from;
    QMap<uint8_t, uint64_t> packets_lost_to;
    double avg_link_quality = 1.0;
    double min_link_quality = 1.0;
    int active_links = 0;
    int total_neighbors = 0;
    QString radio_type;  // "WiFi6E", "5G", "LoRa"
    int channel = 0;
    int bandwidth_mhz = 20;
    double tx_power_dbm = 20.0;
    QDateTime last_update;
};

struct SwarmSafetyEvent {
    uint64_t timestamp_us = 0;
    uint8_t source_id = 0;
    QString event_type;  // "GEOFENCE_BREACH", "ESTOP", "COLLISION_WARNING", "LOW_BATTERY", "GPS_LOSS", "LINK_LOSS"
    QString severity;    // "INFO", "WARN", "ERROR", "CRITICAL"
    QString description;
    QVariantMap details;
    bool acknowledged = false;
    QDateTime acknowledged_time;
    uint8_t acknowledged_by = 0;
};

class SwarmMonitorPlugin : public QGCPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.qgroundcontrol.QGCPlugin" FILE "SwarmMonitor.json")
    Q_INTERFACES(QGCPlugin)

public:
    explicit SwarmMonitorPlugin(QGCApplication* app, QGCToolbox* toolbox);
    ~SwarmMonitorPlugin() override;

    // QGCPlugin interface
    QString name() const override { return "SwarmMonitor"; }
    QString version() const override { return "1.0.0"; }
    QString description() const override { return tr("Real-time swarm monitoring: formation, health, mesh QoS, safety"); }
    QString author() const override { return "AI Swarm Team"; }
    QString repository() const override { return "https://github.com/krill0188/my-drone-project"; }
    
    bool init() override;
    void start() override;
    void stop() override;
    
    // Tool registration
    QList<QGCPanelWidget*> createToolWidgets() override;
    QList<QGCPanelWidget*> createCustomWidgets() override;
    
    // Data access
    Q_INVOKABLE QVariantMap getVehicleStatus(uint8_t vehicle_id) const;
    Q_INVOKABLE QVariantList getAllVehicleStatus() const;
    Q_INVOKABLE QVariantMap getFormationStatus() const;
    Q_INVOKABLE QVariantMap getMeshQoS(uint8_t node_id) const;
    Q_INVOKABLE QVariantList getAllMeshQoS() const;
    Q_INVOKABLE QVariantList getSafetyEvents(int limit = 100) const;
    Q_INVOKABLE QVariantMap getSwarmSummary() const;
    
    // Control
    Q_INVOKABLE bool acknowledgeSafetyEvent(uint64_t timestamp_us, uint8_t ack_by);
    Q_INVOKABLE bool setFormationType(const QString& formation_type);
    Q_INVOKABLE bool requestFormationChange(const QVariantMap& formation_config);
    Q_INVOKABLE bool sendEstop(uint8_t target_id = 0);  // 0 = broadcast
    Q_INVOKABLE bool sendRTL(uint8_t target_id = 0);
    Q_INVOKABLE bool sendHold(uint8_t target_id = 0);
    
    // Python backend
    Q_INVOKABLE QVariantMap runPythonAnalysis(const QString& analysis_type, const QVariantMap& args);
    Q_INVOKABLE bool isPythonBackendAvailable() const;

signals:
    void vehicleStatusUpdated(uint8_t vehicle_id, const QVariantMap& status);
    void vehicleAdded(uint8_t vehicle_id);
    void vehicleRemoved(uint8_t vehicle_id);
    void formationStatusUpdated(const QVariantMap& formation);
    void meshQoSUpdated(uint8_t node_id, const QVariantMap& qos);
    void safetyEventReceived(const QVariantMap& event);
    void safetyEventAcknowledged(uint64_t timestamp_us, uint8_t ack_by);
    void swarmSummaryUpdated(const QVariantMap& summary);
    void pythonBackendStatusChanged(bool available);
    void errorOccurred(const QString& message);

private:
    void setupConnections();
    void setupTimers();
    void updateVehicleStatus(uint8_t vehicle_id);
    void updateFormationStatus();
    void updateMeshQoS();
    void processIncomingMessages();
    void loadSafetyEventsFromStorage();
    void saveSafetyEventsToStorage();
    
    // Data storage
    QMap<uint8_t, SwarmVehicleStatus> _vehicle_status;
    SwarmFormationStatus _formation_status;
    QMap<uint8_t, MeshQoSMetrics> _mesh_qos;
    QVector<SwarmSafetyEvent> _safety_events;
    const int MAX_SAFETY_EVENTS = 10000;
    
    // Python backend
    class PythonBackendBridge;
    PythonBackendBridge* _python_backend = nullptr;
    
    // Timers
    QTimer* _update_timer = nullptr;
    QTimer* _cleanup_timer = nullptr;
    
    // Storage
    QString _storage_path;
};

}  // namespace swarm::gcs

Q_DECLARE_METATYPE(swarm::gcs::SwarmVehicleStatus)
Q_DECLARE_METATYPE(swarm::gcs::SwarmFormationStatus)
Q_DECLARE_METATYPE(swarm::gcs::MeshQoSMetrics)
Q_DECLARE_METATYPE(swarm::gcs::SwarmSafetyEvent)

#endif // SWARM_MONITOR_PLUGIN_H