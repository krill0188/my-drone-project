#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <vector>
#include <map>

class TestAIAnalyzer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testDetectionsGrouping() {
        std::vector<std::tuple<std::string, double, double>> inferences = {
            {"person", 0.9, 30}, {"car", 0.8, 25}, {"person", 0.95, 35},
            {"bike", 0.7, 40}, {"person", 0.85, 28}, {"car", 0.9, 22}
        };
        
        std::map<std::string, std::vector<double>> by_class_conf;
        std::map<std::string, std::vector<double>> by_class_time;
        
        for (auto& [cls, conf, time] : inferences) {
            by_class_conf[cls].push_back(conf);
            by_class_time[cls].push_back(time);
        }
        
        QCOMPARE(by_class_conf["person"].size(), 3);
        QCOMPARE(by_class_conf["car"].size(), 2);
        QCOMPARE(by_class_conf["bike"].size(), 1);
        
        double person_avg_conf = std::accumulate(by_class_conf["person"].begin(), by_class_conf["person"].end(), 0.0) / 3;
        QVERIFY(qFuzzyCompare(person_avg_conf, 0.9, 0.01));
        
        double person_avg_time = std::accumulate(by_class_time["person"].begin(), by_class_time["person"].end(), 0.0) / 3;
        QVERIFY(qFuzzyCompare(person_avg_time, 31.0, 0.1));
    }
    
    void testInferenceTimeStats() {
        std::vector<double> times = {25, 30, 35, 40, 45, 50, 55, 60, 100, 150};
        
        double mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
        QVERIFY(qFuzzyCompare(mean, 49.0, 0.1));
        
        double max_t = *std::max_element(times.begin(), times.end());
        QCOMPARE(max_t, 150.0);
        
        double min_t = *std::min_element(times.begin(), times.end());
        QCOMPARE(min_t, 25.0);
    }
    
    void testEventTimeline() {
        struct Event {
            uint64_t timestamp_us;
            std::string type;
            std::string class_name;
            double confidence;
        };
        
        std::vector<Event> events = {
            {1000000, "detection", "person", 0.9},
            {2000000, "detection", "car", 0.8},
            {3000000, "replan", "", 0},
            {4000000, "detection", "person", 0.95},
            {5000000, "fault", "", 0},
        };
        
        int replan_count = 0, fault_count = 0, detect_count = 0;
        for (auto& e : events) {
            if (e.type == "replan") replan_count++;
            else if (e.type == "fault") fault_count++;
            else if (e.type == "detection") detect_count++;
        }
        
        QCOMPARE(replan_count, 1);
        QCOMPARE(fault_count, 1);
        QCOMPARE(detect_count, 3);
    }
    
    void testInferenceTimeline() {
        std::vector<std::pair<uint64_t, double>> timeline = {
            {1000000, 30}, {2000000, 35}, {3000000, 28},
            {4000000, 45}, {5000000, 50}, {6000000, 120},  // Spike
        };
        
        double mean = 0;
        for (auto& [t, inf] : timeline) mean += inf;
        mean /= timeline.size();
        
        // Detect anomaly (spike at 120ms)
        double threshold = 50;  // Real-time threshold
        int anomalies = 0;
        for (auto& [t, inf] : timeline) {
            if (inf > threshold) anomalies++;
        }
        
        QCOMPARE(anomalies, 1);
    }
};

QTEST_MAIN(TestAIAnalyzer)
#include "test_ai_analyzer.moc"