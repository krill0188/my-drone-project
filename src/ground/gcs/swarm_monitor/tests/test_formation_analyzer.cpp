#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <cmath>
#include <vector>
#include <algorithm>

class TestFormationAnalyzer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testHaversineDistance() {
        auto haversine = [](double lat1, double lon1, double lat2, double lon2) {
            const double R = 6371000;
            double dlat = (lat2 - lat1) * M_PI / 180.0;
            double dlon = (lon2 - lon1) * M_PI / 180.0;
            double a = sin(dlat/2)*sin(dlat/2) + cos(lat1*M_PI/180)*cos(lat2*M_PI/180)*sin(dlon/2)*sin(dlon/2);
            return 2 * R * atan2(sqrt(a), sqrt(1-a));
        };
        
        // Same point
        double d1 = haversine(37.7749, -122.4194, 37.7749, -122.4194);
        QVERIFY(qFuzzyCompare(d1, 0.0));
        
        // ~111km (1 degree latitude)
        double d2 = haversine(37.7749, -122.4194, 38.7749, -122.4194);
        QVERIFY(qFuzzyCompare(d2, 111195.0, 1000.0));  // ~111km with 1km tolerance
        
        // ~111km * cos(lat) (1 degree longitude at lat 37.77)
        double d3 = haversine(37.7749, -122.4194, 37.7749, -121.4194);
        double expected = 111195.0 * cos(37.7749 * M_PI / 180.0);
        QVERIFY(qFuzzyCompare(d3, expected, 1000.0));
    }
    
    void testFormationErrorCalculation() {
        // Vehicle at assigned position
        double error1 = 0.0;  // Perfect
        
        // Vehicle 10m off
        double error2 = 10.0;
        
        // Vehicle 100m off
        double error3 = 100.0;
        
        std::vector<double> errors = {error1, error2, error3};
        
        double avg = 0;
        for (double e : errors) avg += e;
        avg /= errors.size();
        
        QVERIFY(qFuzzyCompare(avg, 36.67, 0.1));
        
        double max_err = *std::max_element(errors.begin(), errors.end());
        QCOMPARE(max_err, 100.0);
        
        // Formation state classification
        auto classify = [](double avg, double max, int lost, int degraded, int total) {
            if (lost > 0) return "lost";
            if (degraded > total * 0.3) return "degraded";
            if (avg < 1.0) return "excellent";
            if (avg < 2.0) return "good";
            return "holding";
        };
        
        QCOMPARE(classify(0.5, 1.0, 0, 0, 3), "excellent");
        QCOMPARE(classify(1.5, 3.0, 0, 0, 3), "good");
        QCOMPARE(classify(3.0, 6.0, 0, 1, 3), "degraded");
        QCOMPARE(classify(10.0, 50.0, 1, 0, 3), "lost");
    }
    
    void testStabilityScore() {
        // Score = max(0, 100 - avg_error * 10)
        auto score = [](double avg) { return std::max(0.0, 100.0 - avg * 10.0); };
        
        QCOMPARE(score(0.0), 100.0);
        QCOMPARE(score(1.0), 90.0);
        QCOMPARE(score(5.0), 50.0);
        QCOMPARE(score(10.0), 0.0);
        QCOMPARE(score(15.0), 0.0);
    }
    
    void testInToleranceCounts() {
        std::vector<double> errors = {0.5, 1.5, 2.5, 4.5, 6.0};
        
        int in_tol = 0, degraded = 0, lost = 0;
        for (double e : errors) {
            if (e < 2.0) in_tol++;
            else if (e < 5.0) degraded++;
            else lost++;
        }
        
        QCOMPARE(in_tol, 2);   // 0.5, 1.5
        QCOMPARE(degraded, 2); // 2.5, 4.5
        QCOMPARE(lost, 1);     // 6.0
    }
};

QTEST_MAIN(TestFormationAnalyzer)
#include "test_formation_analyzer.moc"