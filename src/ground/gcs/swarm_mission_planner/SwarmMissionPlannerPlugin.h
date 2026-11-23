#ifndef SWARM_MISSION_PLANNER_PLUGIN_H
#define SWARM_MISSION_PLANNER_PLUGIN_H

#include <QGCPlugin.h>
#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>
#include <QGeoCoordinate>
#include <QDateTime>

class MissionManager;
class VehicleManager;
class LinkManager;
class QGCApplication;
class QmlObjectListModel;
class SwarmMissionManager;
class SwarmCBBAAllocator;
class PythonBackendBridge;

namespace swarm::gcs {

struct SwarmWaypoint {
    uint32_t sequence = 0;
    QGeoCoordinate coordinate;
    double altitude_m = 0.0;
    uint16_t command = 16;  // MAV_CMD_NAV_WAYPOINT
    QMap<int, double> params;
    bool is_current = false;
    bool autocontinue = true;
    
    // Swarm-specific
    uint8_t assigned_drone_id = 0;  // 0 = unassigned, will be filled by CBBA
    QString task_type;              // "waypoint", "search", "inspect", "loiter", "rtl"
    QString task_id;
};

struct SwarmSearchPolygon {
    QString id;
    QString name;
    QVector<QGeoCoordinate> vertices;  // Polygon vertices (lat/lon)
    double search_altitude_m = 50.0;
    double spacing_m = 20.0;           // Sweep line spacing
    double angle_deg = 0.0;            // Search pattern angle
    QString pattern_type;              // "grid", "spiral", "creeping_line"
    
    // CBBA allocation result
    QMap<uint8_t, QVector<SwarmWaypoint>> drone_waypoints;  // drone_id -> waypoints
    QDateTime created_time;
    QDateTime updated_time;
};

struct SwarmInspectTarget {
    QString id;
    QString name;
    QGeoCoordinate position;
    double inspect_altitude_m = 10.0;  // Relative to target
    double standoff_distance_m = 5.0;
    QVector<double> yaw_angles_deg;    // Multiple view angles
    double gimbal_pitch_deg = -90.0;   // Nadir by default
    QString target_type;               // "structure", "tower", "bridge", "solar_panel", "custom"
    QString priority;                  // "high", "medium", "low"
    
    // CBBA allocation
    uint8_t assigned_drone_id = 0;
    QVector<SwarmWaypoint> approach_waypoints;
    QVector<SwarmWaypoint> inspect_waypoints;
    QVector<SwarmWaypoint> depart_waypoints;
};

struct SwarmMission {
    QString id;
    QString name;
    QString description;
    QDateTime created_time;
    QDateTime updated_time;
    QString created_by;
    
    QVector<SwarmWaypoint> waypoints;
    QVector<SwarmSearchPolygon> search_polygons;
    QVector<SwarmInspectTarget> inspect_targets;
    
    // Swarm configuration
    QVector<uint8_t> participating_drones;  // Drone IDs in this mission
    QString allocation_strategy;            // "cbba", "greedy", "manual"
    bool auto_allocate = true;
    
    // Execution state
    enum class State { Draft, Ready, Uploading, Active, Paused, Completed, Aborted };
    State state = State::Draft;
    uint8_t current_drone_id = 0;
    uint32_t current_waypoint_seq = 0;
};

class SwarmMissionPlannerPlugin : public QGCPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.qgroundcontrol.QGCPlugin" FILE "SwarmMissionPlanner.json")
    Q_INTERFACES(QGCPlugin)

public:
    explicit SwarmMissionPlannerPlugin(QGCApplication* app, QGCToolbox* toolbox);
    ~SwarmMissionPlannerPlugin() override;

    // QGCPlugin interface
    QString name() const override { return "SwarmMissionPlanner"; }
    QString version() const override { return "1.0.0"; }
    QString description() const override { return tr("Multi-drone swarm mission planning with CBBA allocation"); }
    QString author() const override { return "AI Swarm Team"; }
    QString repository() const override { return "https://github.com/krill0188/my-drone-project"; }
    
    bool init() override;
    void start() override;
    void stop() override;
    
    // Tool registration
    QList<QGCPanelWidget*> createToolWidgets() override;
    QList<QGCPanelWidget*> createCustomWidgets() override;
    
    // Mission management
    SwarmMissionManager* missionManager() const { return _mission_manager; }
    SwarmCBBAAllocator* cbbaAllocator() const { return _cbba_allocator; }
    PythonBackendBridge* pythonBackend() const { return _python_backend; }
    
    // Public API for QML
    Q_INVOKABLE QString createMission(const QString& name, const QString& description = "");
    Q_INVOKABLE bool loadMission(const QString& mission_id);
    Q_INVOKABLE bool saveMission(const QString& mission_id);
    Q_INVOKABLE bool deleteMission(const QString& mission_id);
    Q_INVOKABLE QVariantList listMissions() const;
    Q_INVOKABLE QVariantMap getMission(const QString& mission_id) const;
    
    // Waypoint operations
    Q_INVOKABLE QString addWaypoint(const QString& mission_id, const QVariantMap& waypoint_data);
    Q_INVOKABLE bool updateWaypoint(const QString& mission_id, const QString& waypoint_id, const QVariantMap& data);
    Q_INVOKABLE bool removeWaypoint(const QString& mission_id, const QString& waypoint_id);
    Q_INVOKABLE bool reorderWaypoints(const QString& mission_id, const QVariantList& waypoint_ids);
    
    // Search polygon operations
    Q_INVOKABLE QString addSearchPolygon(const QString& mission_id, const QVariantMap& polygon_data);
    Q_INVOKABLE bool updateSearchPolygon(const QString& mission_id, const QString& polygon_id, const QVariantMap& data);
    Q_INVOKABLE bool removeSearchPolygon(const QString& mission_id, const QString& polygon_id);
    Q_INVOKABLE QVariantMap generateSearchPattern(const QVariantMap& polygon_data);
    
    // Inspect target operations
    Q_INVOKABLE QString addInspectTarget(const QString& mission_id, const QVariantMap& target_data);
    Q_INVOKABLE bool updateInspectTarget(const QString& mission_id, const QString& target_id, const QVariantMap& data);
    Q_INVOKABLE bool removeInspectTarget(const QString& mission_id, const QString& target_id);
    Q_INVOKABLE QVariantMap generateInspectWaypoints(const QVariantMap& target_data);
    
    // CBBA Allocation
    Q_INVOKABLE bool runCBBAAllocation(const QString& mission_id);
    Q_INVOKABLE QVariantMap getAllocationResult(const QString& mission_id) const;
    Q_INVOKABLE bool applyAllocation(const QString& mission_id, const QVariantMap& allocation);
    Q_INVOKABLE QVariantMap previewAllocation(const QString& mission_id);
    
    // Mission upload/execution
    Q_INVOKABLE bool uploadMission(const QString& mission_id);
    Q_INVOKABLE bool uploadMissionToDrone(const QString& mission_id, uint8_t drone_id);
    Q_INVOKABLE bool startMission(const QString& mission_id);
    Q_INVOKABLE bool pauseMission(const QString& mission_id);
    Q_INVOKABLE bool resumeMission(const QString& mission_id);
    Q_INVOKABLE bool abortMission(const QString& mission_id);
    
    // Python backend integration (for MP features)
    Q_INVOKABLE QVariantMap runPythonScript(const QString& script_name, const QVariantMap& args);
    Q_INVOKABLE bool isPythonBackendAvailable() const;

signals:
    void missionCreated(const QString& mission_id);
    void missionUpdated(const QString& mission_id);
    void missionDeleted(const QString& mission_id);
    void missionLoaded(const QString& mission_id);
    void missionStateChanged(const QString& mission_id, int state);
    void allocationCompleted(const QString& mission_id, const QVariantMap& result);
    void allocationProgress(int progress, const QString& message);
    void uploadProgress(uint8_t drone_id, int progress, const QString& message);
    void pythonBackendStatusChanged(bool available);
    void errorOccurred(const QString& message);

private:
    void setupConnections();
    void loadMissionsFromStorage();
    void saveMissionsToStorage();
    
    SwarmMissionManager* _mission_manager = nullptr;
    SwarmCBBAAllocator* _cbba_allocator = nullptr;
    PythonBackendBridge* _python_backend = nullptr;
    
    QMap<QString, SwarmMission> _missions;
    QString _active_mission_id;
    QString _storage_path;
};

}  // namespace swarm::gcs

Q_DECLARE_METATYPE(swarm::gcs::SwarmMission)
Q_DECLARE_METATYPE(swarm::gcs::SwarmWaypoint)
Q_DECLARE_METATYPE(swarm::gcs::SwarmSearchPolygon)
Q_DECLARE_METATYPE(swarm::gcs::SwarmInspectTarget)

#endif // SWARM_MISSION_PLANNER_PLUGIN_H