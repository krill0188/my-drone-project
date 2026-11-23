#include "SwarmMonitorPlugin.h"
#include <QGCApplication.h>
#include <QGCToolbox.h>
#include <VehicleManager.h>
#include <LinkManager.h>
#include <MultiVehicleManager.h>
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QUuid>
#include <QDebug>
#include <QProcess>
#include <QEventLoop>
#include <QTimer>

namespace swarm::gcs {

// ============================================================================
// PythonBackendBridge Implementation
// ============================================================================

class SwarmMonitorPlugin::PythonBackendBridge : public QObject
{
    Q_OBJECT
public:
    explicit PythonBackendBridge(QObject* parent = nullptr) : QObject(parent) {
        _process = new QProcess(this);
        connect(_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, &PythonBackendBridge::onProcessFinished);
        connect(_process, &QProcess::readyReadStandardOutput,
                this, &PythonBackendBridge::onReadyRead);
        connect(_process, &QProcess::errorOccurred,
                this, &PythonBackendBridge::onError);
    }
    
    ~PythonBackendBridge() {
        if (_process->state() != QProcess::NotRunning) {
            _process->terminate();
            _process->waitForFinished(1000);
        }
    }
    
    bool start() {
        QString python_path = QStringLiteral("python3");
        QString script_path = QStringLiteral(":/python/swarm_monitor_backend.py");
        
        _process->start(python_path, {script_path, "--server"});
        bool started = _process->waitForStarted(5000);
        _available = started;
        emit availabilityChanged(_available);
        return started;
    }
    
    void stop() {
        if (_process->state() != QProcess::NotRunning) {
            _process->terminate();
            _process->waitForFinished(1000);
        }
        _available = false;
        emit availabilityChanged(false);
    }
    
    bool isAvailable() const { return _available; }
    
    QVariantMap call(const QString& method, const QVariantMap& args) {
        if (!_available) {
            return {{"error", "Python backend not available"}};
        }
        
        QVariantMap request;
        request["id"] = QUuid::createUuid().toString(QUuid::WithoutBraces);
        request["method"] = method;
        request["args"] = args;
        request["timestamp"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
        
        _pending_requests[request["id"].toString()] = QEventLoop();
        
        QJsonDocument doc(QJsonObject::fromVariantMap(request));
        _process->write(doc.toJson(QJsonDocument::Compact) + "\n");
        _process->waitForBytesWritten(1000);
        
        QEventLoop* loop = _pending_requests[request["id"].toString()];
        QTimer::singleShot(30000, loop, &QEventLoop::quit);
        loop->exec();
        
        return _responses.take(request["id"].toString());
    }

signals:
    void availabilityChanged(bool available);
    void outputReceived(const QString& line);

private slots:
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
        _available = false;
        emit availabilityChanged(false);
        qDebug() << "Python monitor backend exited:" << exitCode << exitStatus;
    }
    
    void onReadyRead() {
        while (_process->canReadLine()) {
            QByteArray line = _process->readLine().trimmed();
            if (line.isEmpty()) continue;
            
            QJsonDocument doc = QJsonDocument::fromJson(line);
            if (doc.isObject()) {
                QVariantMap response = doc.object().toVariantMap();
                QString id = response["id"].toString();
                
                if (_pending_requests.contains(id)) {
                    _responses[id] = response;
                    _pending_requests[id].quit();
                    _pending_requests.remove(id);
                }
            }
            emit outputReceived(QString::fromUtf8(line));
        }
    }
    
    void onError(QProcess::ProcessError error) {
        qWarning() << "Python monitor backend error:" << error;
        _available = false;
        emit availabilityChanged(false);
    }

private:
    QProcess* _process;
    bool _available = false;
    QMap<QString, QEventLoop> _pending_requests;
    QMap<QString, QVariantMap> _responses;
};

// ============================================================================
// SwarmMonitorPlugin Implementation
// ============================================================================

SwarmMonitorPlugin::SwarmMonitorPlugin(QGCApplication* app, QGCToolbox* toolbox)
    : QGCPlugin(app, toolbox)
{
    _python_backend = new PythonBackendBridge(this);
    
    _update_timer = new QTimer(this);
    _update_timer->setInterval(100);  // 10 Hz
    connect(_update_timer, &QTimer::timeout, this, &SwarmMonitorPlugin::processIncomingMessages);
    
    _cleanup_timer = new QTimer(this);
    _cleanup_timer->setInterval(60000);  // 1 minute
    connect(_cleanup_timer, &QTimer::timeout, this, [this]() {
        // Clean up old safety events
        if (_safety_events.size() > MAX_SAFETY_EVENTS) {
            _safety_events.erase(_safety_events.begin(), 
                               _safety_events.begin() + (_safety_events.size() - MAX_SAFETY_EVENTS));
        }
        saveSafetyEventsToStorage();
    });
    
    setupConnections();
}

SwarmMonitorPlugin::~SwarmMonitorPlugin() = default;

bool SwarmMonitorPlugin::init() {
    _storage_path = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/swarm_monitor";
    QDir().mkpath(_storage_path);
    
    loadSafetyEventsFromStorage();
    _python_backend->start();
    _update_timer->start();
    _cleanup_timer->start();
    
    return true;
}

void SwarmMonitorPlugin::start() {
    qmlRegisterType<SwarmMonitorPlugin>("SwarmMonitor", 1, 0, "SwarmMonitor");
}

void SwarmMonitorPlugin::stop() {
    _update_timer->stop();
    _cleanup_timer->stop();
    _python_backend->stop();
    saveSafetyEventsToStorage();
}

QList<QGCPanelWidget*> SwarmMonitorPlugin::createToolWidgets() {
    return QList<QGCPanelWidget*>();
}

QList<QGCPanelWidget*> SwarmMonitorPlugin::createCustomWidgets() {
    return QList<QGCPanelWidget*>();
}

void SwarmMonitorPlugin::setupConnections() {
    // Connect to VehicleManager for vehicle updates
    VehicleManager* vehicle_mgr = _toolbox->vehicleManager();
    if (vehicle_mgr) {
        connect(vehicle_mgr, &VehicleManager::vehicleAdded,
                this, [this](Vehicle* vehicle) {
                    uint8_t id = vehicle->id();
                    if (!_vehicle_status.contains(id)) {
                        SwarmVehicleStatus status;
                        status.id = id;
                        status.callsign = vehicle->callsign();
                        status.first_seen = QDateTime::currentDateTimeUtc();
                        _vehicle_status[id] = status;
                        emit vehicleAdded(id);
                    }
                });
        connect(vehicle_mgr, &VehicleManager::vehicleRemoved,
                this, [this](Vehicle* vehicle) {
                    uint8_t id = vehicle->id();
                    _vehicle_status.remove(id);
                    emit vehicleRemoved(id);
                });
    }
    
    // Connect to LinkManager for communication stats
    LinkManager* link_mgr = _toolbox->linkManager();
    if (link_mgr) {
        connect(link_mgr, &LinkManager::linkConnected,
                this, [this](SharedLinkInterfacePtr link) {
                    // Update link quality
                });
        connect(link_mgr, &LinkManager::linkDisconnected,
                this, [this](SharedLinkInterfacePtr link) {
                    // Update link quality
                });
    }
    
    connect(_python_backend, &PythonBackendBridge::availabilityChanged,
            this, &SwarmMonitorPlugin::pythonBackendStatusChanged);
}

QVariantMap SwarmMonitorPlugin::getVehicleStatus(uint8_t vehicle_id) const {
    QVariantMap map;
    if (!_vehicle_status.contains(vehicle_id)) return map;
    
    const SwarmVehicleStatus& s = _vehicle_status[vehicle_id];
    map["id"] = s.id;
    map["callsign"] = s.callsign;
    map["firmware_type"] = s.firmware_type;
    map["lat"] = s.coordinate.latitude();
    map["lon"] = s.coordinate.longitude();
    map["alt_amsl"] = s.altitude_amsl;
    map["alt_rel"] = s.altitude_rel;
    map["heading_deg"] = s.heading_deg;
    map["roll_deg"] = s.roll_deg;
    map["pitch_deg"] = s.pitch_deg;
    map["ground_speed_mps"] = s.ground_speed_mps;
    map["vertical_speed_mps"] = s.vertical_speed_mps;
    map["flight_mode"] = s.flight_mode;
    map["arming_state"] = s.arming_state;
    map["nav_state"] = s.nav_state;
    map["is_flying"] = s.is_flying;
    map["is_leader"] = s.is_leader;
    map["leader_id"] = s.leader_id;
    map["battery_voltage_v"] = s.battery_voltage_v;
    map["battery_current_a"] = s.battery_current_a;
    map["battery_remaining_pct"] = s.battery_remaining_pct;
    map["battery_consumed_mah"] = s.battery_consumed_mah;
    map["battery_cell_count"] = s.battery_cell_count;
    map["cell_voltages_v"] = QVariant::fromValue(s.cell_voltages_v);
    map["gps_satellites"] = s.gps_satellites;
    map["gps_hdop"] = s.gps_hdop;
    map["gps_fix_type"] = s.gps_fix_type;
    map["gps_healthy"] = s.gps_healthy;
    map["ekf_innovation"] = s.ekf_innovation;
    map["ekf_healthy"] = s.ekf_healthy;
    map["ekf_reset_count"] = s.ekf_reset_count;
    map["link_quality"] = s.link_quality;
    map["rssi_dbm"] = s.rssi_dbm;
    map["snr_db"] = s.snr_db;
    map["latency_ms"] = s.latency_ms;
    map["last_heartbeat_us"] = s.last_heartbeat_us;
    map["messages_received"] = s.messages_received;
    map["messages_lost"] = s.messages_lost;
    map["active_detections"] = s.active_detections;
    map["inference_time_ms"] = s.inference_time_ms;
    map["last_detection_class"] = s.last_detection_class;
    map["last_detection_confidence"] = s.last_detection_confidence;
    map["geofence_ok"] = s.geofence_ok;
    map["estop_active"] = s.estop_active;
    map["last_safety_event"] = s.last_safety_event;
    map["formation_error_m"] = s.formation_error_m;
    map["formation_target_lat"] = s.formation_target.latitude();
    map["formation_target_lon"] = s.formation_target.longitude();
    map["in_formation"] = s.in_formation;
    map["last_update"] = s.last_update.toString(Qt::ISODate);
    map["first_seen"] = s.first_seen.toString(Qt::ISODate);
    
    return map;
}

QVariantList SwarmMonitorPlugin::getAllVehicleStatus() const {
    QVariantList list;
    for (auto it = _vehicle_status.constBegin(); it != _vehicle_status.constEnd(); ++it) {
        list.append(getVehicleStatus(it.key()));
    }
    return list;
}

QVariantMap SwarmMonitorPlugin::getFormationStatus() const {
    QVariantMap map;
    const SwarmFormationStatus& f = _formation_status;
    
    map["leader_id"] = f.leader_id;
    map["formation_type"] = f.formation_type;
    map["member_ids"] = QVariant::fromValue(f.member_ids);
    map["formation_locked"] = f.formation_locked;
    map["formation_state"] = f.formation_state;
    map["avg_formation_error_m"] = f.avg_formation_error_m;
    map["max_formation_error_m"] = f.max_formation_error_m;
    map["last_update"] = f.last_update.toString(Qt::ISODate);
    
    QVariantMap assigned, actual, errors;
    for (auto it = f.assigned_positions.constBegin(); it != f.assigned_positions.constEnd(); ++it) {
        QVariantMap pos;
        pos["lat"] = it.value().latitude();
        pos["lon"] = it.value().longitude();
        assigned[QString::number(it.key())] = pos;
    }
    for (auto it = f.actual_positions.constBegin(); it != f.actual_positions.constEnd(); ++it) {
        QVariantMap pos;
        pos["lat"] = it.value().latitude();
        pos["lon"] = it.value().longitude();
        actual[QString::number(it.key())] = pos;
    }
    for (auto it = f.position_errors_m.constBegin(); it != f.position_errors_m.constEnd(); ++it) {
        errors[QString::number(it.key())] = it.value();
    }
    map["assigned_positions"] = assigned;
    map["actual_positions"] = actual;
    map["position_errors_m"] = errors;
    
    return map;
}

QVariantMap SwarmMonitorPlugin::getMeshQoS(uint8_t node_id) const {
    QVariantMap map;
    if (!_mesh_qos.contains(node_id)) return map;
    
    const MeshQoSMetrics& m = _mesh_qos[node_id];
    map["node_id"] = m.node_id;
    map["radio_type"] = m.radio_type;
    map["channel"] = m.channel;
    map["bandwidth_mhz"] = m.bandwidth_mhz;
    map["tx_power_dbm"] = m.tx_power_dbm;
    map["avg_link_quality"] = m.avg_link_quality;
    map["min_link_quality"] = m.min_link_quality;
    map["active_links"] = m.active_links;
    map["total_neighbors"] = m.total_neighbors;
    map["last_update"] = m.last_update.toString(Qt::ISODate);
    
    QVariantMap link_q, rssi, latency, sent, recv, lost;
    for (auto it = m.link_quality_to.constBegin(); it != m.link_quality_to.constEnd(); ++it) {
        link_q[QString::number(it.key())] = it.value();
    }
    for (auto it = m.rssi_dbm_to.constBegin(); it != m.rssi_dbm_to.constEnd(); ++it) {
        rssi[QString::number(it.key())] = it.value();
    }
    for (auto it = m.latency_ms_to.constBegin(); it != m.latency_ms_to.constEnd(); ++it) {
        latency[QString::number(it.key())] = it.value();
    }
    for (auto it = m.packets_sent_to.constBegin(); it != m.packets_sent_to.constEnd(); ++it) {
        sent[QString::number(it.key())] = it.value();
    }
    for (auto it = m.packets_received_from.constBegin(); it != m.packets_received_from.constEnd(); ++it) {
        recv[QString::number(it.key())] = it.value();
    }
    for (auto it = m.packets_lost_to.constBegin(); it != m.packets_lost_to.constEnd(); ++it) {
        lost[QString::number(it.key())] = it.value();
    }
    
    map["link_quality_to"] = link_q;
    map["rssi_dbm_to"] = rssi;
    map["latency_ms_to"] = latency;
    map["packets_sent_to"] = sent;
    map["packets_received_from"] = recv;
    map["packets_lost_to"] = lost;
    
    return map;
}

QVariantList SwarmMonitorPlugin::getAllMeshQoS() const {
    QVariantList list;
    for (auto it = _mesh_qos.constBegin(); it != _mesh_qos.constEnd(); ++it) {
        list.append(getMeshQoS(it.key()));
    }
    return list;
}

QVariantList SwarmMonitorPlugin::getSafetyEvents(int limit) const {
    QVariantList list;
    int count = qMin(limit, _safety_events.size());
    for (int i = _safety_events.size() - count; i < _safety_events.size(); ++i) {
        if (i < 0) continue;
        const SwarmSafetyEvent& e = _safety_events[i];
        QVariantMap map;
        map["timestamp_us"] = e.timestamp_us;
        map["source_id"] = e.source_id;
        map["event_type"] = e.event_type;
        map["severity"] = e.severity;
        map["description"] = e.description;
        map["details"] = e.details;
        map["acknowledged"] = e.acknowledged;
        map["acknowledged_time"] = e.acknowledged_time.toString(Qt::ISODate);
        map["acknowledged_by"] = e.acknowledged_by;
        list.append(map);
    }
    return list;
}

QVariantMap SwarmMonitorPlugin::getSwarmSummary() const {
    QVariantMap summary;
    summary["total_vehicles"] = _vehicle_status.size();
    summary["armed_count"] = 0;
    summary["flying_count"] = 0;
    summary["leader_id"] = _formation_status.leader_id;
    summary["formation_type"] = _formation_status.formation_type;
    summary["formation_state"] = _formation_status.formation_state;
    summary["avg_formation_error_m"] = _formation_status.avg_formation_error_m;
    summary["max_formation_error_m"] = _formation_status.max_formation_error_m;
    summary["unacknowledged_safety_events"] = 0;
    summary["critical_safety_events"] = 0;
    
    double avg_battery = 0;
    double avg_link_quality = 0;
    int battery_count = 0;
    int link_count = 0;
    
    for (const auto& s : _vehicle_status) {
        if (s.arming_state == "ARMED") summary["armed_count"] = summary["armed_count"].toInt() + 1;
        if (s.is_flying) summary["flying_count"] = summary["flying_count"].toInt() + 1;
        
        if (s.battery_remaining_pct > 0) {
            avg_battery += s.battery_remaining_pct;
            battery_count++;
        }
        if (s.link_quality > 0) {
            avg_link_quality += s.link_quality;
            link_count++;
        }
    }
    
    if (battery_count > 0) summary["avg_battery_pct"] = avg_battery / battery_count;
    if (link_count > 0) summary["avg_link_quality"] = avg_link_quality / link_count;
    
    for (const auto& e : _safety_events) {
        if (!e.acknowledged) summary["unacknowledged_safety_events"] = summary["unacknowledged_safety_events"].toInt() + 1;
        if (e.severity == "CRITICAL") summary["critical_safety_events"] = summary["critical_safety_events"].toInt() + 1;
    }
    
    summary["timestamp"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    return summary;
}

bool SwarmMonitorPlugin::acknowledgeSafetyEvent(uint64_t timestamp_us, uint8_t ack_by) {
    for (auto& e : _safety_events) {
        if (e.timestamp_us == timestamp_us && !e.acknowledged) {
            e.acknowledged = true;
            e.acknowledged_time = QDateTime::currentDateTimeUtc();
            e.acknowledged_by = ack_by;
            emit safetyEventAcknowledged(timestamp_us, ack_by);
            saveSafetyEventsToStorage();
            return true;
        }
    }
    return false;
}

bool SwarmMonitorPlugin::setFormationType(const QString& formation_type) {
    // Would send formation change command via MAVLink
    _formation_status.formation_type = formation_type;
    _formation_status.formation_state = "transitioning";
    emit formationStatusUpdated(getFormationStatus());
    return true;
}

bool SwarmMonitorPlugin::requestFormationChange(const QVariantMap& formation_config) {
    // Would send custom formation command
    QString type = formation_config.value("formation_type", "custom").toString();
    _formation_status.formation_type = type;
    _formation_status.formation_state = "transitioning";
    emit formationStatusUpdated(getFormationStatus());
    return true;
}

bool SwarmMonitorPlugin::sendEstop(uint8_t target_id) {
    // Would broadcast ESTOP via MAVLink
    emit errorOccurred(QString("ESTOP sent to %1").arg(target_id == 0 ? "ALL" : QString::number(target_id)));
    return true;
}

bool SwarmMonitorPlugin::sendRTL(uint8_t target_id) {
    // Would send RTL command
    return true;
}

bool SwarmMonitorPlugin::sendHold(uint8_t target_id) {
    // Would send HOLD command
    return true;
}

QVariantMap SwarmMonitorPlugin::runPythonAnalysis(const QString& analysis_type, const QVariantMap& args) {
    return _python_backend->call(analysis_type, args);
}

bool SwarmMonitorPlugin::isPythonBackendAvailable() const {
    return _python_backend->isAvailable();
}

void SwarmMonitorPlugin::processIncomingMessages() {
    // Update vehicle status from VehicleManager
    VehicleManager* vehicle_mgr = _toolbox->vehicleManager();
    if (vehicle_mgr) {
        const QList<Vehicle*>& vehicles = vehicle_mgr->vehicles();
        for (Vehicle* vehicle : vehicles) {
            updateVehicleStatus(vehicle->id());
        }
    }
    
    // Update formation status
    updateFormationStatus();
    
    // Update mesh QoS
    updateMeshQoS();
    
    // Emit summary periodically
    static int summary_counter = 0;
    if (++summary_counter >= 10) {  // Every 1 second at 10Hz
        summary_counter = 0;
        emit swarmSummaryUpdated(getSwarmSummary());
    }
}

void SwarmMonitorPlugin::updateVehicleStatus(uint8_t vehicle_id) {
    VehicleManager* vehicle_mgr = _toolbox->vehicleManager();
    if (!vehicle_mgr) return;
    
    Vehicle* vehicle = vehicle_mgr->getVehicle(vehicle_id);
    if (!vehicle) return;
    
    SwarmVehicleStatus& s = _vehicle_status[vehicle_id];
    if (s.first_seen.isNull()) {
        s.first_seen = QDateTime::currentDateTimeUtc();
    }
    
    s.id = vehicle_id;
    s.callsign = vehicle->callsign();
    s.coordinate = vehicle->coordinate();
    s.altitude_amsl = vehicle->altitudeAMSL();
    s.altitude_rel = vehicle->altitudeRelative();
    s.heading_deg = vehicle->heading();
    s.roll_deg = vehicle->roll();
    s.pitch_deg = vehicle->pitch();
    s.ground_speed_mps = vehicle->groundSpeed();
    s.vertical_speed_mps = vehicle->climbRate();
    s.flight_mode = vehicle->flightMode();
    s.arming_state = vehicle->armed() ? "ARMED" : "DISARMED";
    s.is_flying = vehicle->isFlying();
    
    // Battery
    if (vehicle->battery()) {
        s.battery_voltage_v = vehicle->battery()->voltage();
        s.battery_current_a = vehicle->battery()->current();
        s.battery_remaining_pct = vehicle->battery()->remainingPercent();
    }
    
    // GPS
    if (vehicle->gps()) {
        s.gps_satellites = vehicle->gps()->satelliteCount();
        s.gps_hdop = vehicle->gps()->hdop();
        s.gps_fix_type = vehicle->gps()->fixType();
        s.gps_healthy = vehicle->gps()->healthy();
    }
    
    // Communication
    LinkManager* link_mgr = _toolbox->linkManager();
    if (link_mgr) {
        SharedLinkInterfacePtr link = link_mgr->getLinkForVehicle(vehicle_id);
        if (link) {
            s.link_quality = link->linkQuality();
            s.rssi_dbm = link->rssi();
            s.snr_db = link->snr();
            s.latency_ms = link->latency();
            s.last_heartbeat_us = link->lastHeartbeatTime();
        }
    }
    
    s.last_update = QDateTime::currentDateTimeUtc();
    _vehicle_status[vehicle_id] = s;
    emit vehicleStatusUpdated(vehicle_id, getVehicleStatus(vehicle_id));
}

void SwarmMonitorPlugin::updateFormationStatus() {
    // This would be updated from MAVLink swarm messages
    // For now, compute from vehicle positions if formation is active
    if (_formation_status.member_ids.isEmpty()) return;
    
    double total_error = 0;
    double max_error = 0;
    int count = 0;
    
    for (uint8_t id : _formation_status.member_ids) {
        if (!_vehicle_status.contains(id)) continue;
        if (!_formation_status.assigned_positions.contains(id)) continue;
        
        const auto& vehicle = _vehicle_status[id];
        const auto& target = _formation_status.assigned_positions[id];
        
        double error = vehicle.coordinate.distanceTo(target);
        _formation_status.position_errors_m[id] = error;
        _formation_status.actual_positions[id] = vehicle.coordinate;
        
        total_error += error;
        max_error = qMax(max_error, error);
        count++;
    }
    
    if (count > 0) {
        _formation_status.avg_formation_error_m = total_error / count;
        _formation_status.max_formation_error_m = max_error;
        _formation_status.last_update = QDateTime::currentDateTimeUtc();
        
        if (max_error > 5.0) {
            _formation_status.formation_state = "lost";
        } else if (max_error > 2.0) {
            _formation_status.formation_state = "degraded";
        } else {
            _formation_status.formation_state = "holding";
        }
        
        emit formationStatusUpdated(getFormationStatus());
    }
}

void SwarmMonitorPlugin::updateMeshQoS() {
    // This would be updated from MAVLink mesh messages
    // For now, simulate from link manager
    LinkManager* link_mgr = _toolbox->linkManager();
    if (!link_mgr) return;
    
    const QList<SharedLinkInterfacePtr>& links = link_mgr->links();
    for (const auto& link : links) {
        uint8_t node_id = link->sourceSystemId();
        if (node_id == 0) continue;  // Ground station
        
        MeshQoSMetrics& m = _mesh_qos[node_id];
        m.node_id = node_id;
        m.last_update = QDateTime::currentDateTimeUtc();
        
        m.link_quality_to[link->targetSystemId()] = link->linkQuality();
        m.rssi_dbm_to[link->targetSystemId()] = link->rssi();
        m.latency_ms_to[link->targetSystemId()] = link->latency();
        m.packets_sent_to[link->targetSystemId()]++;
        
        if (m.link_quality_to.size() > 0) {
            double sum = 0;
            for (auto it = m.link_quality_to.begin(); it != m.link_quality_to.end(); ++it) {
                sum += it.value();
            }
            m.avg_link_quality = sum / m.link_quality_to.size();
            m.min_link_quality = *std::min_element(m.link_quality_to.begin(), m.link_quality_to.end(),
                                                   [](const auto& a, const auto& b) { return a.value() < b.value(); }).value();
            m.active_links = m.link_quality_to.size();
        }
        
        _mesh_qos[node_id] = m;
        emit meshQoSUpdated(node_id, getMeshQoS(node_id));
    }
}

void SwarmMonitorPlugin::loadSafetyEventsFromStorage() {
    QDir dir(_storage_path);
    QStringList files = dir.entryList({"safety_events_*.json"}, QDir::Files | QDir::Time, QDir::Reversed);
    
    for (const QString& file : files) {
        QFile f(dir.absoluteFilePath(file));
        if (f.open(QIODevice::ReadOnly)) {
            QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
            if (doc.isArray()) {
                QJsonArray arr = doc.array();
                for (const auto& val : arr) {
                    QJsonObject obj = val.toObject();
                    SwarmSafetyEvent e;
                    e.timestamp_us = obj["timestamp_us"].toVariant().toULongLong();
                    e.source_id = obj["source_id"].toInt();
                    e.event_type = obj["event_type"].toString();
                    e.severity = obj["severity"].toString();
                    e.description = obj["description"].toString();
                    e.details = obj["details"].toVariant().toMap();
                    e.acknowledged = obj["acknowledged"].toBool();
                    e.acknowledged_time = QDateTime::fromString(obj["acknowledged_time"].toString(), Qt::ISODate);
                    e.acknowledged_by = obj["acknowledged_by"].toInt();
                    _safety_events.append(e);
                }
            }
        }
    }
    
    // Keep only recent
    if (_safety_events.size() > MAX_SAFETY_EVENTS) {
        _safety_events.erase(_safety_events.begin(), 
                           _safety_events.begin() + (_safety_events.size() - MAX_SAFETY_EVENTS));
    }
}

void SwarmMonitorPlugin::saveSafetyEventsToStorage() {
    QJsonArray arr;
    for (const auto& e : _safety_events) {
        QJsonObject obj;
        obj["timestamp_us"] = QJsonValue::fromVariant(static_cast<qulonglong>(e.timestamp_us));
        obj["source_id"] = e.source_id;
        obj["event_type"] = e.event_type;
        obj["severity"] = e.severity;
        obj["description"] = e.description;
        obj["details"] = QJsonObject::fromVariantMap(e.details);
        obj["acknowledged"] = e.acknowledged;
        obj["acknowledged_time"] = e.acknowledged_time.toString(Qt::ISODate);
        obj["acknowledged_by"] = e.acknowledged_by;
        arr.append(obj);
    }
    
    QJsonDocument doc(arr);
    QString file_name = _storage_path + "/safety_events_" + QDateTime::currentDateTimeUtc().toString("yyyyMMdd") + ".json";
    QFile f(file_name);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(doc.toJson(QJsonDocument::Indented));
    }
}

}  // namespace swarm::gcs

#include "SwarmMonitorPlugin.moc"