#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <vector>
#include <algorithm>
#include <numeric>

class TestFormationAnalyzer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testCDFCalculation() {
        std::vector<double> errors = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 10.0};
        std::sort(errors.begin(), errors.end());
        
        std::vector<double> cdf_x = errors;
        std::vector<double> cdf_y;
        for (size_t i = 0; i < errors.size(); ++i) {
            cdf_y.push_back((i + 1) / double(errors.size()));
        }
        
        QCOMPARE(cdf_x.size(), cdf_y.size());
        QVERIFY(cdf_y.back() == 1.0);
        QVERIFY(cdf_y.front() == 0.1);
        
        // P95
        size_t p95_idx = size_t(0.95 * errors.size());
        QCOMPARE(errors[p95_idx], 6.0);
        
        // P99
        size_t p99_idx = size_t(0.99 * errors.size());
        QCOMPARE(errors[p99_idx], 10.0);
    }
    
    void testFormationStateClassification() {
        auto classify = [](double avg, double max, int lost, int degraded, int total) {
            if (lost > 0) return "lost";
            if (degraded > total * 0.3) return "degraded";
            if (avg < 1.0) return "excellent";
            if (avg < 2.0) return "good";
            return "holding";
        };
        
        QCOMPARE(classify(0.5, 1.0, 0, 0, 10), "excellent");
        QCOMPARE(classify(1.5, 3.0, 0, 0, 10), "good");
        QCOMPARE(classify(3.0, 6.0, 0, 4, 10), "degraded");
        QCOMPARE(classify(10.0, 50.0, 1, 0, 10), "lost");
    }
    
    void testToleranceCounts() {
        std::vector<double> errors = {0.5, 1.0, 1.5, 2.5, 3.0, 4.5, 6.0};
        
        int in_tol = 0, degraded = 0, lost = 0;
        for (double e : errors) {
            if (e < 2.0) in_tol++;
            else if (e < 5.0) degraded++;
            else lost++;
        }
        
        QCOMPARE(in_tol, 3);
        QCOMPARE(degraded, 3);
        QCOMPARE(lost, 1);
    }
    
    void testStatistics() {
        std::vector<double> errors = {1.0, 2.0, 3.0, 4.0, 5.0};
        
        double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
        QVERIFY(qFuzzyCompare(mean, 3.0));
        
        double variance = 0;
        for (double e : errors) {
            variance += (e - mean) * (e - mean);
        }
        variance /= errors.size();
        double std = sqrt(variance);
        QVERIFY(qFuzzyCompare(std, sqrt(2.0), 0.01));
        
        std::sort(errors.begin(), errors.end());
        double median = errors[errors.size() / 2];
        QCOMPARE(median, 3.0);
        
        double max_e = *std::max_element(errors.begin(), errors.end());
        QCOMPARE(max_e, 5.0);
    }
};

QTEST_MAIN(TestFormationAnalyzer)
#include "test_formation_analyzer.moc"