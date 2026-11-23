#include <QtTest/QtTest>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <QtPositioning/QGeoCoordinate>

#include "SwarmMissionPlannerPlugin.h"

using namespace swarm::gcs;

class TestMissionManager : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {
        // Initialize plugin (mock)
    }
    
    void cleanupTestCase() {
    }
    
    void testCreateMission() {
        // Mock test - would need actual plugin instance
        SwarmMission mission;
        mission.id = "test-mission-1";
        mission.name = "Test Mission";
        mission.description = "Test description";
        mission.created_time = QDateTime::currentDateTimeUtc();
        mission.state = SwarmMission::State::Draft;
        
        QVERIFY(!mission.id.isEmpty());
        QCOMPARE(mission.name, QString("Test Mission"));
        QCOMPARE(mission.state, SwarmMission::State::Draft);
    }
    
    void testWaypointSerialization() {
        SwarmWaypoint wp;
        wp.sequence = 0;
        wp.coordinate = QGeoCoordinate(37.7749, -122.4194, 50.0);
        wp.altitude_m = 50.0;
        wp.command = 16;
        wp.assigned_drone_id = 1;
        wp.task_type = "waypoint";
        wp.task_id = "task-1";
        
        QVariantMap map;
        map["sequence"] = wp.sequence;
        map["lat"] = wp.coordinate.latitude();
        map["lon"] = wp.coordinate.longitude();
        map["alt"] = wp.altitude_m;
        map["command"] = wp.command;
        map["assigned_drone_id"] = wp.assigned_drone_id;
        map["task_type"] = wp.task_type;
        map["task_id"] = wp.task_id;
        
        QCOMPARE(map["sequence"].toInt(), 0);
        QVERIFY(qFuzzyCompare(map["lat"].toDouble(), 37.7749));
        QVERIFY(qFuzzyCompare(map["lon"].toDouble(), -122.4194));
        QCOMPARE(map["alt"].toDouble(), 50.0);
        QCOMPARE(map["assigned_drone_id"].toInt(), 1);
    }
    
    void testSearchPolygonSerialization() {
        SwarmSearchPolygon sp;
        sp.id = "search-1";
        sp.name = "Test Search Area";
        sp.vertices = {
            QGeoCoordinate(37.7749, -122.4194),
            QGeoCoordinate(37.7849, -122.4194),
            QGeoCoordinate(37.7849, -122.4094),
            QGeoCoordinate(37.7749, -122.4094)
        };
        sp.search_altitude_m = 50.0;
        sp.spacing_m = 20.0;
        sp.angle_deg = 0.0;
        sp.pattern_type = "grid";
        
        QVariantList vertices;
        for (const auto& v : sp.vertices) {
            QVariantMap v_map;
            v_map["lat"] = v.latitude();
            v_map["lon"] = v.longitude();
            vertices.append(v_map);
        }
        
        QCOMPARE(vertices.size(), 4);
        QVERIFY(qFuzzyCompare(vertices[0].toMap()["lat"].toDouble(), 37.7749));
    }
    
    void testInspectTargetSerialization() {
        SwarmInspectTarget it;
        it.id = "inspect-1";
        it.name = "Tower Inspection";
        it.position = QGeoCoordinate(37.7749, -122.4194);
        it.inspect_altitude_m = 10.0;
        it.standoff_distance_m = 5.0;
        it.yaw_angles_deg = {0.0, 90.0, 180.0, 270.0};
        it.gimbal_pitch_deg = -90.0;
        it.target_type = "tower";
        it.priority = "high";
        
        QVariantMap map;
        map["id"] = it.id;
        map["name"] = it.name;
        map["lat"] = it.position.latitude();
        map["lon"] = it.position.longitude();
        map["inspect_altitude_m"] = it.inspect_altitude_m;
        map["standoff_distance_m"] = it.standoff_distance_m;
        map["yaw_angles_deg"] = QVariant::fromValue(it.yaw_angles_deg);
        map["gimbal_pitch_deg"] = it.gimbal_pitch_deg;
        map["target_type"] = it.target_type;
        map["priority"] = it.priority;
        
        QCOMPARE(map["id"].toString(), "inspect-1");
        QCOMPARE(map["yaw_angles_deg"].toList().size(), 4);
    }
    
    void testMissionStateTransitions() {
        SwarmMission mission;
        mission.state = SwarmMission::State::Draft;
        
        // Draft -> Ready
        mission.state = SwarmMission::State::Ready;
        QCOMPARE(mission.state, SwarmMission::State::Ready);
        
        // Ready -> Active
        mission.state = SwarmMission::State::Active;
        QCOMPARE(mission.state, SwarmMission::State::Active);
        
        // Active -> Paused
        mission.state = SwarmMission::State::Paused;
        QCOMPARE(mission.state, SwarmMission::State::Paused);
        
        // Paused -> Active
        mission.state = SwarmMission::State::Active;
        QCOMPARE(mission.state, SwarmMission::State::Active);
        
        // Active -> Completed
        mission.state = SwarmMission::State::Completed;
        QCOMPARE(mission.state, SwarmMission::State::Completed);
        
        // Any -> Aborted
        mission.state = SwarmMission::State::Aborted;
        QCOMPARE(mission.state, SwarmMission::State::Aborted);
    }
    
    void testJsonSerialization() {
        SwarmMission mission;
        mission.id = "json-test";
        mission.name = "JSON Test";
        mission.waypoints = {
            {0, QGeoCoordinate(37.7749, -122.4194, 50), 50.0, 16, {}, false, true, 1, "waypoint", "wp1"}
        };
        mission.search_polygons = {
            {"sp1", "Search 1", {QGeoCoordinate(37.7749, -122.4194)}, 50, 20, 0, "grid", {}, QDateTime::currentDateTimeUtc(), QDateTime::currentDateTimeUtc()}
        };
        mission.inspect_targets = {
            {"it1", "Target 1", QGeoCoordinate(37.7749, -122.4194), 10, 5, {0, 90}, -90, "tower", "high", 1, {}, {}, {}}
        };
        
        // Serialize to JSON
        QJsonObject obj;
        obj["id"] = mission.id;
        obj["name"] = mission.name;
        
        QJsonArray wp_array;
        for (const auto& wp : mission.waypoints) {
            QJsonObject wp_obj;
            wp_obj["sequence"] = wp.sequence;
            wp_obj["lat"] = wp.coordinate.latitude();
            wp_obj["lon"] = wp.coordinate.longitude();
            wp_obj["alt"] = wp.altitude_m;
            wp_array.append(wp_obj);
        }
        obj["waypoints"] = wp_array;
        
        QJsonDocument doc(obj);
        QString json_str = doc.toJson(QJsonDocument::Compact);
        
        // Deserialize
        QJsonDocument doc2 = QJsonDocument::fromJson(json_str.toUtf8());
        QJsonObject obj2 = doc2.object();
        
        QCOMPARE(obj2["id"].toString(), "json-test");
        QCOMPARE(obj2["name"].toString(), "JSON Test");
        QCOMPARE(obj2["waypoints"].toArray().size(), 1);
    }
};

QTEST_MAIN(TestMissionManager)
#include "test_mission_manager.moc"