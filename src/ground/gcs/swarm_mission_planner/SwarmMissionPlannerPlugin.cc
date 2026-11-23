#include "SwarmMissionPlannerPlugin.h"
#include <QGCApplication.h>
#include <QGCToolbox.h>
#include <MissionManager.h>
#include <VehicleManager.h>
#include <LinkManager.h>
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
#include <QTimer>

namespace swarm::gcs {

// ============================================================================
// SwarmMissionManager Implementation
// ============================================================================

class SwarmMissionManager : public QObject
{
    Q_OBJECT
public:
    explicit SwarmMissionManager(SwarmMissionPlannerPlugin* parent) : QObject(parent), _plugin(parent) {}
    
    QString createMission(const QString& name, const QString& description) {
        SwarmMission mission;
        mission.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
        mission.name = name;
        mission.description = description;
        mission.created_time = QDateTime::currentDateTimeUtc();
        mission.updated_time = mission.created_time;
        mission.state = SwarmMission::State::Draft;
        
        _missions[mission.id] = mission;
        emit missionCreated(mission.id);
        return mission.id;
    }
    
    bool loadMission(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        _active_mission_id = mission_id;
        emit missionLoaded(mission_id);
        return true;
    }
    
    bool saveMission(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        _missions[mission_id].updated_time = QDateTime::currentDateTimeUtc();
        emit missionUpdated(mission_id);
        return true;
    }
    
    bool deleteMission(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        _missions.remove(mission_id);
        if (_active_mission_id == mission_id) _active_mission_id.clear();
        emit missionDeleted(mission_id);
        return true;
    }
    
    QVariantList listMissions() const {
        QVariantList list;
        for (auto it = _missions.constBegin(); it != _missions.constEnd(); ++it) {
            const SwarmMission& m = it.value();
            QVariantMap map;
            map["id"] = m.id;
            map["name"] = m.name;
            map["description"] = m.description;
            map["created_time"] = m.created_time.toString(Qt::ISODate);
            map["updated_time"] = m.updated_time.toString(Qt::ISODate);
            map["state"] = static_cast<int>(m.state);
            map["waypoint_count"] = m.waypoints.size();
            map["search_polygon_count"] = m.search_polygons.size();
            map["inspect_target_count"] = m.inspect_targets.size();
            map["drone_count"] = m.participating_drones.size();
            list.append(map);
        }
        return list;
    }
    
    QVariantMap getMission(const QString& mission_id) const {
        QVariantMap map;
        if (!_missions.contains(mission_id)) return map;
        
        const SwarmMission& m = _missions[mission_id];
        map["id"] = m.id;
        map["name"] = m.name;
        map["description"] = m.description;
        map["created_time"] = m.created_time.toString(Qt::ISODate);
        map["updated_time"] = m.updated_time.toString(Qt::ISODate);
        map["state"] = static_cast<int>(m.state);
        map["allocation_strategy"] = m.allocation_strategy;
        map["auto_allocate"] = m.auto_allocate;
        map["participating_drones"] = QVariant::fromValue(m.participating_drones);
        
        // Waypoints
        QVariantList wp_list;
        for (const auto& wp : m.waypoints) {
            QVariantMap wp_map;
            wp_map["sequence"] = wp.sequence;
            wp_map["lat"] = wp.coordinate.latitude();
            wp_map["lon"] = wp.coordinate.longitude();
            wp_map["alt"] = wp.altitude_m;
            wp_map["command"] = wp.command;
            wp_map["params"] = QVariant::fromValue(wp.params);
            wp_map["is_current"] = wp.is_current;
            wp_map["autocontinue"] = wp.autocontinue;
            wp_map["assigned_drone_id"] = wp.assigned_drone_id;
            wp_map["task_type"] = wp.task_type;
            wp_map["task_id"] = wp.task_id;
            wp_list.append(wp_map);
        }
        map["waypoints"] = wp_list;
        
        // Search polygons
        QVariantList sp_list;
        for (const auto& sp : m.search_polygons) {
            QVariantMap sp_map;
            sp_map["id"] = sp.id;
            sp_map["name"] = sp.name;
            QVariantList vertices;
            for (const auto& v : sp.vertices) {
                QVariantMap v_map;
                v_map["lat"] = v.latitude();
                v_map["lon"] = v.longitude();
                vertices.append(v_map);
            }
            sp_map["vertices"] = vertices;
            sp_map["search_altitude_m"] = sp.search_altitude_m;
            sp_map["spacing_m"] = sp.spacing_m;
            sp_map["angle_deg"] = sp.angle_deg;
            sp_map["pattern_type"] = sp.pattern_type;
            sp_map["drone_waypoints"] = serializeDroneWaypoints(sp.drone_waypoints);
            sp_map["created_time"] = sp.created_time.toString(Qt::ISODate);
            sp_map["updated_time"] = sp.updated_time.toString(Qt::ISODate);
            sp_list.append(sp_map);
        }
        map["search_polygons"] = sp_list;
        
        // Inspect targets
        QVariantList it_list;
        for (const auto& it : m.inspect_targets) {
            QVariantMap it_map;
            it_map["id"] = it.id;
            it_map["name"] = it.name;
            it_map["lat"] = it.position.latitude();
            it_map["lon"] = it.position.longitude();
            it_map["inspect_altitude_m"] = it.inspect_altitude_m;
            it_map["standoff_distance_m"] = it.standoff_distance_m;
            it_map["yaw_angles_deg"] = QVariant::fromValue(it.yaw_angles_deg);
            it_map["gimbal_pitch_deg"] = it.gimbal_pitch_deg;
            it_map["target_type"] = it.target_type;
            it_map["priority"] = it.priority;
            it_map["assigned_drone_id"] = it.assigned_drone_id;
            it_map["approach_waypoints"] = serializeWaypoints(it.approach_waypoints);
            it_map["inspect_waypoints"] = serializeWaypoints(it.inspect_waypoints);
            it_map["depart_waypoints"] = serializeWaypoints(it.depart_waypoints);
            it_list.append(it_map);
        }
        map["inspect_targets"] = it_list;
        
        return map;
    }
    
    QString addWaypoint(const QString& mission_id, const QVariantMap& data) {
        if (!_missions.contains(mission_id)) return "";
        
        SwarmMission& m = _missions[mission_id];
        SwarmWaypoint wp;
        wp.sequence = m.waypoints.size();
        wp.coordinate = QGeoCoordinate(data["lat"].toDouble(), data["lon"].toDouble());
        wp.altitude_m = data["alt"].toDouble();
        wp.command = data.value("command", 16).toInt();
        wp.params = data.value("params").toMap().toStdMap();  // Simplified
        wp.is_current = data.value("is_current", false).toBool();
        wp.autocontinue = data.value("autocontinue", true).toBool();
        wp.assigned_drone_id = data.value("assigned_drone_id", 0).toInt();
        wp.task_type = data.value("task_type", "waypoint").toString();
        wp.task_id = data.value("task_id", "").toString();
        
        m.waypoints.append(wp);
        m.updated_time = QDateTime::currentDateTimeUtc();
        emit missionUpdated(mission_id);
        return QString::number(wp.sequence);
    }
    
    bool updateWaypoint(const QString& mission_id, const QString& wp_id, const QVariantMap& data) {
        if (!_missions.contains(mission_id)) return false;
        
        SwarmMission& m = _missions[mission_id];
        int seq = wp_id.toInt();
        if (seq < 0 || seq >= m.waypoints.size()) return false;
        
        SwarmWaypoint& wp = m.waypoints[seq];
        if (data.contains("lat") && data.contains("lon")) {
            wp.coordinate = QGeoCoordinate(data["lat"].toDouble(), data["lon"].toDouble());
        }
        if (data.contains("alt")) wp.altitude_m = data["alt"].toDouble();
        if (data.contains("command")) wp.command = data["command"].toInt();
        if (data.contains("assigned_drone_id")) wp.assigned_drone_id = data["assigned_drone_id"].toInt();
        if (data.contains("task_type")) wp.task_type = data["task_type"].toString();
        if (data.contains("task_id")) wp.task_id = data["task_id"].toString();
        
        m.updated_time = QDateTime::currentDateTimeUtc();
        emit missionUpdated(mission_id);
        return true;
    }
    
    bool removeWaypoint(const QString& mission_id, const QString& wp_id) {
        if (!_missions.contains(mission_id)) return false;
        
        SwarmMission& m = _missions[mission_id];
        int seq = wp_id.toInt();
        if (seq < 0 || seq >= m.waypoints.size()) return false;
        
        m.waypoints.removeAt(seq);
        // Renumber sequences
        for (int i = 0; i < m.waypoints.size(); ++i) {
            m.waypoints[i].sequence = i;
        }
        m.updated_time = QDateTime::currentDateTimeUtc();
        emit missionUpdated(mission_id);
        return true;
    }
    
    QString addSearchPolygon(const QString& mission_id, const QVariantMap& data) {
        if (!_missions.contains(mission_id)) return "";
        
        SwarmMission& m = _missions[mission_id];
        SwarmSearchPolygon sp;
        sp.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
        sp.name = data.value("name", "Search Area").toString();
        
        QVariantList vertices = data.value("vertices").toList();
        for (const auto& v : vertices) {
            QVariantMap v_map = v.toMap();
            sp.vertices.append(QGeoCoordinate(v_map["lat"].toDouble(), v_map["lon"].toDouble()));
        }
        
        sp.search_altitude_m = data.value("search_altitude_m", 50.0).toDouble();
        sp.spacing_m = data.value("spacing_m", 20.0).toDouble();
        sp.angle_deg = data.value("angle_deg", 0.0).toDouble();
        sp.pattern_type = data.value("pattern_type", "grid").toString();
        sp.created_time = QDateTime::currentDateTimeUtc();
        sp.updated_time = sp.created_time;
        
        m.search_polygons.append(sp);
        m.updated_time = QDateTime::currentDateTimeUtc();
        emit missionUpdated(mission_id);
        return sp.id;
    }
    
    QVariantMap generateSearchPattern(const QVariantMap& data) {
        // Generate lawnmower/grid pattern for polygon
        // Simplified implementation - would use proper geometry library
        QVariantMap result;
        QVariantList waypoints;
        
        QVariantList vertices = data.value("vertices").toList();
        if (vertices.size() < 3) {
            result["error"] = "Polygon must have at least 3 vertices";
            return result;
        }
        
        double altitude = data.value("search_altitude_m", 50.0).toDouble();
        double spacing = data.value("spacing_m", 20.0).toDouble();
        double angle = data.value("angle_deg", 0.0).toDouble();
        QString pattern = data.value("pattern_type", "grid").toString();
        
        // Mock pattern generation
        for (int i = 0; i < 10; ++i) {
            QVariantMap wp;
            wp["lat"] = vertices[0].toMap()["lat"].toDouble() + i * 0.0001;
            wp["lon"] = vertices[0].toMap()["lon"].toDouble() + i * 0.0001;
            wp["alt"] = altitude;
            wp["command"] = 16;
            waypoints.append(wp);
        }
        
        result["waypoints"] = waypoints;
        result["pattern_type"] = pattern;
        return result;
    }
    
    QString addInspectTarget(const QString& mission_id, const QVariantMap& data) {
        if (!_missions.contains(mission_id)) return "";
        
        SwarmMission& m = _missions[mission_id];
        SwarmInspectTarget it;
        it.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
        it.name = data.value("name", "Inspect Target").toString();
        it.position = QGeoCoordinate(data["lat"].toDouble(), data["lon"].toDouble());
        it.inspect_altitude_m = data.value("inspect_altitude_m", 10.0).toDouble();
        it.standoff_distance_m = data.value("standoff_distance_m", 5.0).toDouble();
        it.yaw_angles_deg = data.value("yaw_angles_deg", QVariantList{0.0}).toList().toVector().toStdVector();  // Simplified
        it.gimbal_pitch_deg = data.value("gimbal_pitch_deg", -90.0).toDouble();
        it.target_type = data.value("target_type", "structure").toString();
        it.priority = data.value("priority", "medium").toString();
        
        m.inspect_targets.append(it);
        m.updated_time = QDateTime::currentDateTimeUtc();
        emit missionUpdated(mission_id);
        return it.id;
    }
    
    QVariantMap generateInspectWaypoints(const QVariantMap& data) {
        QVariantMap result;
        QVariantList approach, inspect, depart;
        
        QGeoCoordinate pos(data["lat"].toDouble(), data["lon"].toDouble());
        double alt = data.value("inspect_altitude_m", 10.0).toDouble();
        double standoff = data.value("standoff_distance_m", 5.0).toDouble();
        QVariantList yaw_angles = data.value("yaw_angles_deg", QVariantList{0.0}).toList();
        double pitch = data.value("gimbal_pitch_deg", -90.0).toDouble();
        
        // Generate approach waypoint
        QVariantMap wp_approach;
        wp_approach["lat"] = pos.latitude();
        wp_approach["lon"] = pos.longitude();
        wp_approach["alt"] = alt + standoff;
        wp_approach["command"] = 16;
        approach.append(wp_approach);
        
        // Generate inspect waypoints for each yaw angle
        for (const auto& yaw_var : yaw_angles) {
            QVariantMap wp_inspect;
            wp_inspect["lat"] = pos.latitude();
            wp_inspect["lon"] = pos.longitude();
            wp_inspect["alt"] = alt;
            wp_inspect["command"] = 16;  // Would use DO_SET_ROI or similar
            wp_inspect["gimbal_yaw"] = yaw_var.toDouble();
            wp_inspect["gimbal_pitch"] = pitch;
            inspect.append(wp_inspect);
        }
        
        // Depart
        QVariantMap wp_depart;
        wp_depart["lat"] = pos.latitude();
        wp_depart["lon"] = pos.longitude();
        wp_depart["alt"] = alt + standoff;
        wp_depart["command"] = 16;
        depart.append(wp_depart);
        
        result["approach"] = approach;
        result["inspect"] = inspect;
        result["depart"] = depart;
        return result;
    }
    
    bool runCBBAAllocation(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        
        SwarmMission& m = _missions[mission_id];
        if (m.participating_drones.isEmpty()) {
            emit errorOccurred("No drones selected for mission");
            return false;
        }
        
        // This would call the actual CBBA allocator
        // For now, mock allocation
        QVariantMap allocation = previewAllocation(mission_id);
        return applyAllocation(mission_id, allocation);
    }
    
    QVariantMap previewAllocation(const QString& mission_id) {
        QVariantMap result;
        if (!_missions.contains(mission_id)) return result;
        
        const SwarmMission& m = _missions[mission_id];
        QVariantMap drone_assignments;
        
        // Simple round-robin assignment for preview
        int drone_idx = 0;
        for (const auto& wp : m.waypoints) {
            uint8_t drone = m.participating_drones[drone_idx % m.participating_drones.size()];
            QString key = QString::number(drone);
            if (!drone_assignments.contains(key)) {
                drone_assignments[key] = QVariantList();
            }
            QVariantMap wp_map;
            wp_map["sequence"] = wp.sequence;
            wp_map["lat"] = wp.coordinate.latitude();
            wp_map["lon"] = wp.coordinate.longitude();
            wp_map["alt"] = wp.altitude_m;
            drone_assignments[key].toList().append(wp_map);
            drone_idx++;
        }
        
        // Assign search polygons
        for (const auto& sp : m.search_polygons) {
            uint8_t drone = m.participating_drones[drone_idx % m.participating_drones.size()];
            QString key = QString::number(drone);
            if (!drone_assignments.contains(key)) {
                drone_assignments[key] = QVariantList();
            }
            QVariantMap sp_map;
            sp_map["id"] = sp.id;
            sp_map["type"] = "search_polygon";
            drone_assignments[key].toList().append(sp_map);
            drone_idx++;
        }
        
        // Assign inspect targets
        for (const auto& it : m.inspect_targets) {
            uint8_t drone = m.participating_drones[drone_idx % m.participating_drones.size()];
            QString key = QString::number(drone);
            if (!drone_assignments.contains(key)) {
                drone_assignments[key] = QVariantList();
            }
            QVariantMap it_map;
            it_map["id"] = it.id;
            it_map["type"] = "inspect_target";
            drone_assignments[key].toList().append(it_map);
            drone_idx++;
        }
        
        result["assignments"] = drone_assignments;
        result["drones"] = QVariant::fromValue(m.participating_drones);
        return result;
    }
    
    bool applyAllocation(const QString& mission_id, const QVariantMap& allocation) {
        if (!_missions.contains(mission_id)) return false;
        
        SwarmMission& m = _missions[mission_id];
        QVariantMap assignments = allocation.value("assignments").toMap();
        
        // Apply to waypoints
        for (auto it = assignments.begin(); it != assignments.end(); ++it) {
            uint8_t drone_id = it.key().toInt();
            QVariantList tasks = it.value().toList();
            
            for (const auto& task_var : tasks) {
                QVariantMap task = task_var.toMap();
                if (task.value("type").toString() == "waypoint") {
                    int seq = task.value("sequence").toInt();
                    if (seq >= 0 && seq < m.waypoints.size()) {
                        m.waypoints[seq].assigned_drone_id = drone_id;
                    }
                }
            }
        }
        
        m.updated_time = QDateTime::currentDateTimeUtc();
        emit missionUpdated(mission_id);
        emit allocationCompleted(mission_id, allocation);
        return true;
    }
    
    bool uploadMission(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        
        const SwarmMission& m = _missions[mission_id];
        if (m.participating_drones.isEmpty()) {
            emit errorOccurred("No drones selected");
            return false;
        }
        
        // Upload to each drone
        for (uint8_t drone_id : m.participating_drones) {
            uploadMissionToDrone(mission_id, drone_id);
        }
        return true;
    }
    
    bool uploadMissionToDrone(const QString& mission_id, uint8_t drone_id) {
        // Would send mission items via MAVLink to specific drone
        emit uploadProgress(drone_id, 100, "Upload complete");
        return true;
    }
    
    bool startMission(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        SwarmMission& m = _missions[mission_id];
        m.state = SwarmMission::State::Active;
        emit missionStateChanged(mission_id, static_cast<int>(m.state));
        return true;
    }
    
    bool pauseMission(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        SwarmMission& m = _missions[mission_id];
        m.state = SwarmMission::State::Paused;
        emit missionStateChanged(mission_id, static_cast<int>(m.state));
        return true;
    }
    
    bool resumeMission(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        SwarmMission& m = _missions[mission_id];
        m.state = SwarmMission::State::Active;
        emit missionStateChanged(mission_id, static_cast<int>(m.state));
        return true;
    }
    
    bool abortMission(const QString& mission_id) {
        if (!_missions.contains(mission_id)) return false;
        SwarmMission& m = _missions[mission_id];
        m.state = SwarmMission::State::Aborted;
        emit missionStateChanged(mission_id, static_cast<int>(m.state));
        return true;
    }

signals:
    void missionCreated(const QString& mission_id);
    void missionUpdated(const QString& mission_id);
    void missionDeleted(const QString& mission_id);
    void missionLoaded(const QString& mission_id);
    void missionStateChanged(const QString& mission_id, int state);
    void allocationCompleted(const QString& mission_id, const QVariantMap& result);
    void allocationProgress(int progress, const QString& message);
    void uploadProgress(uint8_t drone_id, int progress, const QString& message);
    void errorOccurred(const QString& message);

private:
    SwarmMissionPlannerPlugin* _plugin;
    QMap<QString, SwarmMission> _missions;
    QString _active_mission_id;
    
    QVariantList serializeWaypoints(const QVector<SwarmWaypoint>& waypoints) {
        QVariantList list;
        for (const auto& wp : waypoints) {
            QVariantMap map;
            map["sequence"] = wp.sequence;
            map["lat"] = wp.coordinate.latitude();
            map["lon"] = wp.coordinate.longitude();
            map["alt"] = wp.altitude_m;
            map["command"] = wp.command;
            map["assigned_drone_id"] = wp.assigned_drone_id;
            map["task_type"] = wp.task_type;
            map["task_id"] = wp.task_id;
            list.append(map);
        }
        return list;
    }
    
    QVariantMap serializeDroneWaypoints(const QMap<uint8_t, QVector<SwarmWaypoint>>& drone_wps) {
        QVariantMap map;
        for (auto it = drone_wps.begin(); it != drone_wps.end(); ++it) {
            map[QString::number(it.key())] = serializeWaypoints(it.value());
        }
        return map;
    }
};

// ============================================================================
// SwarmCBBAAllocator Implementation
// ============================================================================

class SwarmCBBAAllocator : public QObject
{
    Q_OBJECT
public:
    explicit SwarmCBBAAllocator(QObject* parent = nullptr) : QObject(parent) {}
    
    QVariantMap allocate(const SwarmMission& mission) {
        QVariantMap result;
        // Full CBBA implementation would go here
        // For now, return mock result
        result["algorithm"] = "CBBA";
        result["iterations"] = 10;
        result["converged"] = true;
        result["assignments"] = QVariantMap();
        return result;
    }

signals:
    void allocationProgress(int progress, const QString& message);
    void allocationCompleted(const QVariantMap& result);
};

// ============================================================================
// PythonBackendBridge Implementation
// ============================================================================

class PythonBackendBridge : public QObject
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
        QString script_path = QStringLiteral(":/python/swarm_mission_backend.py");
        
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
        
        // Wait for response (with timeout)
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
        qDebug() << "Python backend exited:" << exitCode << exitStatus;
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
        qWarning() << "Python backend error:" << error;
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
// SwarmMissionPlannerPlugin Implementation
// ============================================================================

SwarmMissionPlannerPlugin::SwarmMissionPlannerPlugin(QGCApplication* app, QGCToolbox* toolbox)
    : QGCPlugin(app, toolbox)
{
    _mission_manager = new SwarmMissionManager(this);
    _cbba_allocator = new SwarmCBBAAllocator(this);
    _python_backend = new PythonBackendBridge(this);
    
    setupConnections();
}

SwarmMissionPlannerPlugin::~SwarmMissionPlannerPlugin() = default;

bool SwarmMissionPlannerPlugin::init() {
    // Setup storage path
    _storage_path = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/swarm_missions";
    QDir().mkpath(_storage_path);
    
    // Load saved missions
    loadMissionsFromStorage();
    
    // Start Python backend
    _python_backend->start();
    
    return true;
}

void SwarmMissionPlannerPlugin::start() {
    // Register QML types
    qmlRegisterType<SwarmMissionPlannerPlugin>("SwarmMissionPlanner", 1, 0, "SwarmMissionPlanner");
}

void SwarmMissionPlannerPlugin::stop() {
    _python_backend->stop();
    saveMissionsToStorage();
}

QList<QGCPanelWidget*> SwarmMissionPlannerPlugin::createToolWidgets() {
    // Main mission planner widget would be created here
    // Returns a QGCMapToolWidget or similar for the mission planning UI
    return QList<QGCPanelWidget*>();
}

QList<QGCPanelWidget*> SwarmMissionPlannerPlugin::createCustomWidgets() {
    return QList<QGCPanelWidget*>();
}

void SwarmMissionPlannerPlugin::setupConnections() {
    connect(_mission_manager, &SwarmMissionManager::missionCreated,
            this, &SwarmMissionPlannerPlugin::missionCreated);
    connect(_mission_manager, &SwarmMissionManager::missionUpdated,
            this, &SwarmMissionPlannerPlugin::missionUpdated);
    connect(_mission_manager, &SwarmMissionManager::missionDeleted,
            this, &SwarmMissionPlannerPlugin::missionDeleted);
    connect(_mission_manager, &SwarmMissionManager::missionLoaded,
            this, &SwarmMissionPlannerPlugin::missionLoaded);
    connect(_mission_manager, &SwarmMissionManager::missionStateChanged,
            this, &SwarmMissionPlannerPlugin::missionStateChanged);
    connect(_mission_manager, &SwarmMissionManager::allocationCompleted,
            this, &SwarmMissionPlannerPlugin::allocationCompleted);
    connect(_mission_manager, &SwarmMissionManager::allocationProgress,
            this, &SwarmMissionPlannerPlugin::allocationProgress);
    connect(_mission_manager, &SwarmMissionManager::uploadProgress,
            this, &SwarmMissionPlannerPlugin::uploadProgress);
    connect(_mission_manager, &SwarmMissionManager::errorOccurred,
            this, &SwarmMissionPlannerPlugin::errorOccurred);
    connect(_python_backend, &PythonBackendBridge::availabilityChanged,
            this, &SwarmMissionPlannerPlugin::pythonBackendStatusChanged);
}

// Forward all mission manager methods
QString SwarmMissionPlannerPlugin::createMission(const QString& name, const QString& description) {
    return _mission_manager->createMission(name, description);
}

bool SwarmMissionPlannerPlugin::loadMission(const QString& mission_id) {
    return _mission_manager->loadMission(mission_id);
}

bool SwarmMissionPlannerPlugin::saveMission(const QString& mission_id) {
    return _mission_manager->saveMission(mission_id);
}

bool SwarmMissionPlannerPlugin::deleteMission(const QString& mission_id) {
    return _mission_manager->deleteMission(mission_id);
}

QVariantList SwarmMissionPlannerPlugin::listMissions() const {
    return _mission_manager->listMissions();
}

QVariantMap SwarmMissionPlannerPlugin::getMission(const QString& mission_id) const {
    return _mission_manager->getMission(mission_id);
}

QString SwarmMissionPlannerPlugin::addWaypoint(const QString& mission_id, const QVariantMap& data) {
    return _mission_manager->addWaypoint(mission_id, data);
}

bool SwarmMissionPlannerPlugin::updateWaypoint(const QString& mission_id, const QString& wp_id, const QVariantMap& data) {
    return _mission_manager->updateWaypoint(mission_id, wp_id, data);
}

bool SwarmMissionPlannerPlugin::removeWaypoint(const QString& mission_id, const QString& wp_id) {
    return _mission_manager->removeWaypoint(mission_id, wp_id);
}

bool SwarmMissionPlannerPlugin::reorderWaypoints(const QString& mission_id, const QVariantList& waypoint_ids) {
    // Implementation would reorder waypoints
    return _mission_manager->saveMission(mission_id);
}

QString SwarmMissionPlannerPlugin::addSearchPolygon(const QString& mission_id, const QVariantMap& data) {
    return _mission_manager->addSearchPolygon(mission_id, data);
}

bool SwarmMissionPlannerPlugin::updateSearchPolygon(const QString& mission_id, const QString& polygon_id, const QVariantMap& data) {
    // Implementation
    return _mission_manager->saveMission(mission_id);
}

bool SwarmMissionPlannerPlugin::removeSearchPolygon(const QString& mission_id, const QString& polygon_id) {
    // Implementation
    return _mission_manager->saveMission(mission_id);
}

QVariantMap SwarmMissionPlannerPlugin::generateSearchPattern(const QVariantMap& data) {
    return _mission_manager->generateSearchPattern(data);
}

QString SwarmMissionPlannerPlugin::addInspectTarget(const QString& mission_id, const QVariantMap& data) {
    return _mission_manager->addInspectTarget(mission_id, data);
}

bool SwarmMissionPlannerPlugin::updateInspectTarget(const QString& mission_id, const QString& target_id, const QVariantMap& data) {
    return _mission_manager->saveMission(mission_id);
}

bool SwarmMissionPlannerPlugin::removeInspectTarget(const QString& mission_id, const QString& target_id) {
    return _mission_manager->saveMission(mission_id);
}

QVariantMap SwarmMissionPlannerPlugin::generateInspectWaypoints(const QVariantMap& data) {
    return _mission_manager->generateInspectWaypoints(data);
}

bool SwarmMissionPlannerPlugin::runCBBAAllocation(const QString& mission_id) {
    return _mission_manager->runCBBAAllocation(mission_id);
}

QVariantMap SwarmMissionPlannerPlugin::getAllocationResult(const QString& mission_id) const {
    return _mission_manager->previewAllocation(mission_id);
}

bool SwarmMissionPlannerPlugin::applyAllocation(const QString& mission_id, const QVariantMap& allocation) {
    return _mission_manager->applyAllocation(mission_id, allocation);
}

QVariantMap SwarmMissionPlannerPlugin::previewAllocation(const QString& mission_id) {
    return _mission_manager->previewAllocation(mission_id);
}

bool SwarmMissionPlannerPlugin::uploadMission(const QString& mission_id) {
    return _mission_manager->uploadMission(mission_id);
}

bool SwarmMissionPlannerPlugin::uploadMissionToDrone(const QString& mission_id, uint8_t drone_id) {
    return _mission_manager->uploadMissionToDrone(mission_id, drone_id);
}

bool SwarmMissionPlannerPlugin::startMission(const QString& mission_id) {
    return _mission_manager->startMission(mission_id);
}

bool SwarmMissionPlannerPlugin::pauseMission(const QString& mission_id) {
    return _mission_manager->pauseMission(mission_id);
}

bool SwarmMissionPlannerPlugin::resumeMission(const QString& mission_id) {
    return _mission_manager->resumeMission(mission_id);
}

bool SwarmMissionPlannerPlugin::abortMission(const QString& mission_id) {
    return _mission_manager->abortMission(mission_id);
}

QVariantMap SwarmMissionPlannerPlugin::runPythonScript(const QString& script_name, const QVariantMap& args) {
    return _python_backend->call(script_name, args);
}

bool SwarmMissionPlannerPlugin::isPythonBackendAvailable() const {
    return _python_backend->isAvailable();
}

void SwarmMissionPlannerPlugin::loadMissionsFromStorage() {
    QDir dir(_storage_path);
    QStringList files = dir.entryList({"*.json"}, QDir::Files);
    
    for (const QString& file : files) {
        QFile f(dir.absoluteFilePath(file));
        if (f.open(QIODevice::ReadOnly)) {
            QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
            if (doc.isObject()) {
                // Deserialize mission - simplified
                QJsonObject obj = doc.object();
                // Would properly deserialize here
            }
        }
    }
}

void SwarmMissionPlannerPlugin::saveMissionsToStorage() {
    // Serialize all missions to storage
    for (auto it = _mission_manager->_missions.begin(); it != _mission_manager->_missions.end(); ++it) {
        const SwarmMission& m = it.value();
        QString file_path = _storage_path + "/" + m.id + ".json";
        QFile f(file_path);
        if (f.open(QIODevice::WriteOnly)) {
            QJsonObject obj;
            obj["id"] = m.id;
            obj["name"] = m.name;
            // ... serialize all fields
            QJsonDocument doc(obj);
            f.write(doc.toJson(QJsonDocument::Indented));
        }
    }
}

}  // namespace swarm::gcs

#include "SwarmMissionPlannerPlugin.moc"