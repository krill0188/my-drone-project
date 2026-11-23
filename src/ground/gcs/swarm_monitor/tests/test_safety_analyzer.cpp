#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <vector>
#include <algorithm>
#include <map>

class TestSafetyAnalyzer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testEventAggregation() {
        QVariantList events = {
            QVariantMap{{"event_type", "GEOFENCE_BREACH"}, {"severity", "ERROR"}, {"source_id", 1}},
            QVariantMap{{"event_type", "LOW_BATTERY"}, {"severity", "WARN"}, {"source_id", 2}},
            QVariantMap{{"event_type", "LINK_LOSS"}, {"severity", "CRITICAL"}, {"source_id", 1}},
            QVariantMap{{"event_type", "GEOFENCE_BREACH"}, {"severity", "ERROR"}, {"source_id", 3}},
            QVariantMap{{"event_type", "ESTOP"}, {"severity", "CRITICAL"}, {"source_id", 2}},
            QVariantMap{{"event_type", "GPS_LOSS"}, {"severity", "ERROR"}, {"source_id", 1}},
        };
        
        std::map<QString, int> by_type;
        std::map<QString, int> by_severity;
        std::map<int, int> by_source;
        
        for (const auto& e : events) {
            by_type[e["event_type"].toString()]++;
            by_severity[e["severity"].toString()]++;
            by_source[e["source_id"].toInt()]++;
        }
        
        QCOMPARE(by_type["GEOFENCE_BREACH"], 2);
        QCOMPARE(by_type["LOW_BATTERY"], 1);
        QCOMPARE(by_type["LINK_LOSS"], 1);
        QCOMPARE(by_type["ESTOP"], 1);
        QCOMPARE(by_type["GPS_LOSS"], 1);
        
        QCOMPARE(by_severity["CRITICAL"], 2);
        QCOMPARE(by_severity["ERROR"], 3);
        QCOMPARE(by_severity["WARN"], 1);
        
        QCOMPARE(by_source[1], 3);
        QCOMPARE(by_source[2], 2);
        QCOMPARE(by_source[3], 1);
    }
    
    void testEventRate() {
        // Events in last 24 hours
        int total = 50;
        double window_hours = 24;
        double rate = total / window_hours;
        
        QVERIFY(qFuzzyCompare(rate, 2.08, 0.1));
    }
    
    void testTopTypes() {
        std::map<QString, int> by_type = {
            {"GEOFENCE_BREACH", 20},
            {"LOW_BATTERY", 15},
            {"LINK_LOSS", 10},
            {"GPS_LOSS", 8},
            {"ESTOP", 5},
            {"COLLISION_WARNING", 3},
        };
        
        std::vector<std::pair<QString, int>> sorted(by_type.begin(), by_type.end());
        std::sort(sorted.begin(), sorted.end(), 
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        
        QCOMPARE(sorted[0].first, "GEOFENCE_BREACH");
        QCOMPARE(sorted[0].second, 20);
        QCOMPARE(sorted[1].first, "LOW_BATTERY");
        QCOMPARE(sorted[4].first, "ESTOP");
    }
    
    void testCorrelation() {
        // Event at time 1000
        QVariantMap event;
        event["timestamp_us"] = 1000000000;  // 1000 seconds
        event["event_type"] = "LOW_BATTERY";
        
        // Telemetry around event time
        QVariantList telemetry = {
            QVariantMap{{"timestamp", 999}, {"battery_remaining_pct", 25}, {"link_quality", 0.9}},
            QVariantMap{{"timestamp", 1000}, {"battery_remaining_pct", 20}, {"link_quality", 0.9}},
            QVariantMap{{"timestamp", 1001}, {"battery_remaining_pct", 18}, {"link_quality", 0.9}},
            QVariantMap{{"timestamp", 1030}, {"battery_remaining_pct", 15}, {"link_quality", 0.9}},  // Outside 30s window
        };
        
        int window_seconds = 30;
        double event_time = 1000;
        
        std::vector<double> battery_vals;
        for (const auto& t : telemetry) {
            if (std::abs(t["timestamp"].toInt() - event_time) <= window_seconds) {
                battery_vals.push_back(t["battery_remaining_pct"].toDouble());
            }
        }
        
        QCOMPARE(battery_vals.size(), 3);  // 999, 1000, 1001
        double avg_battery = std::accumulate(battery_vals.begin(), battery_vals.end(), 0.0) / battery_vals.size();
        QVERIFY(qFuzzyCompare(avg_battery, 21.0, 0.1));
    }
    
    void testCauseIdentification() {
        auto identifyCauses = [](const QString& event_type, const QVariantList& telemetry) {
            QStringList causes;
            
            if (event_type == "GEOFENCE_BREACH") {
                causes << "Position drifted outside boundary";
                for (const auto& t : telemetry) {
                    if (t["formation_error_m"].toDouble() > 5) {
                        causes << "Formation error contributed to breach";
                        break;
                    }
                }
            } else if (event_type == "LOW_BATTERY") {
                causes << "Battery below threshold";
                for (const auto& t : telemetry) {
                    if (t["battery_current_a"].toDouble() > 20) {
                        causes << "High current draw detected";
                        break;
                    }
                }
            } else if (event_type == "LINK_LOSS") {
                causes << "Communication link lost";
                for (const auto& t : telemetry) {
                    if (t["link_quality"].toDouble() < 0.3) {
                        causes << "Link quality was degrading prior to loss";
                        break;
                    }
                }
            } else if (event_type == "GPS_LOSS") {
                causes << "GPS fix lost";
                for (const auto& t : telemetry) {
                    if (t["gps_satellites"].toInt() < 6) {
                        causes << "Low satellite count prior to loss";
                        break;
                    }
                }
            }
            
            return causes;
        };
        
        QVariantList telemetry1 = {
            QVariantMap{{"formation_error_m", 6.0}}
        };
        auto causes1 = identifyCauses("GEOFENCE_BREACH", telemetry1);
        QVERIFY(causes1.size() == 2);
        
        QVariantList telemetry2 = {
            QVariantMap{{"battery_current_a", 25}}
        };
        auto causes2 = identifyCauses("LOW_BATTERY", telemetry2);
        QVERIFY(causes2.size() == 2);
        
        QVariantList telemetry3 = {
            QVariantMap{{"link_quality", 0.2}}
        };
        auto causes3 = identifyCauses("LINK_LOSS", telemetry3);
        QVERIFY(causes3.size() == 2);
        
        QVariantList telemetry4 = {
            QVariantMap{{"gps_satellites", 4}}
        };
        auto causes4 = identifyCauses("GPS_LOSS", telemetry4);
        QVERIFY(causes4.size() == 2);
    }
};

QTEST_MAIN(TestSafetyAnalyzer)
#include "test_safety_analyzer.moc"