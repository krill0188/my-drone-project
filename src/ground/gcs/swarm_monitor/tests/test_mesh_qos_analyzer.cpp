#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <vector>
#include <algorithm>
#include <numeric>

class TestMeshQoSAnalyzer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testLinkQualityAggregation() {
        // Simulate node data
        struct Link {
            int from, to;
            double quality, rssi, latency, loss_rate;
        };
        
        std::vector<Link> links = {
            {1, 2, 0.9, -50, 10, 0.001},
            {1, 3, 0.7, -70, 25, 0.02},
            {2, 1, 0.9, -50, 10, 0.001},
            {2, 3, 0.8, -60, 15, 0.005},
            {3, 1, 0.7, -70, 25, 0.02},
            {3, 2, 0.8, -60, 15, 0.005},
        };
        
        std::vector<double> qualities, rssis, latencies, loss_rates;
        for (const auto& l : links) {
            qualities.push_back(l.quality);
            rssis.push_back(l.rssi);
            latencies.push_back(l.latency);
            loss_rates.push_back(l.loss_rate);
        }
        
        auto mean = [](const std::vector<double>& v) {
            return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        };
        
        QVERIFY(qFuzzyCompare(mean(qualities), 0.8, 0.01));
        QVERIFY(qFuzzyCompare(mean(rssis), -60.0, 1.0));
        QVERIFY(qFuzzyCompare(mean(latencies), 16.67, 1.0));
        QVERIFY(qFuzzyCompare(mean(loss_rates), 0.0087, 0.001));
    }
    
    void testPercentile() {
        auto percentile = [](std::vector<double> data, double p) {
            if (data.empty()) return 0.0;
            std::sort(data.begin(), data.end());
            double k = (data.size() - 1) * p / 100;
            int f = floor(k);
            int c = ceil(k);
            if (f == c) return data[f];
            return data[f] * (c - k) + data[c] * (k - f);
        };
        
        std::vector<double> latencies = {5, 10, 15, 20, 25, 30, 35, 40, 50, 100};
        
        double p50 = percentile(latencies, 50);
        double p95 = percentile(latencies, 95);
        
        QVERIFY(p50 > 15 && p50 < 25);  // ~22.5
        QVERIFY(p95 > 70 && p95 < 100); // ~97.5
    }
    
    void testCriticalLinkDetection() {
        struct Link {
            int from, to;
            double quality, loss_rate;
        };
        
        std::vector<Link> links = {
            {1, 2, 0.9, 0.001},
            {1, 3, 0.2, 0.15},   // CRITICAL: quality < 0.3
            {2, 1, 0.9, 0.001},
            {2, 3, 0.5, 0.08},   // HIGH: loss > 0.05
            {3, 1, 0.2, 0.15},   // CRITICAL
            {3, 2, 0.5, 0.08},   // HIGH
        };
        
        int critical = 0, high = 0, medium = 0, low = 0;
        for (const auto& l : links) {
            if (l.quality < 0.3 || l.loss_rate > 0.1) critical++;
            else if (l.quality < 0.5 || l.loss_rate > 0.05) high++;
            else if (l.quality < 0.7 || l.loss_rate > 0.01) medium++;
            else low++;
        }
        
        QCOMPARE(critical, 2);  // 1->3, 3->1
        QCOMPARE(high, 2);      // 2->3, 3->2
        QCOMPARE(low, 2);       // 1->2, 2->1
        QCOMPARE(medium, 0);
    }
    
    void testMeshRecommendations() {
        auto generateRecommendations = [](double avg_quality, double min_rssi, double avg_latency, double avg_loss) {
            std::vector<std::string> recs;
            
            if (avg_quality < 0.5) {
                recs.push_back("Average link quality below 50%");
            }
            if (min_rssi < -85) {
                recs.push_back("Minimum RSSI below -85dBm");
            }
            if (avg_latency > 100) {
                recs.push_back("Average latency >100ms");
            }
            if (avg_loss > 0.01) {
                recs.push_back("Packet loss >1%");
            }
            
            return recs;
        };
        
        auto recs1 = generateRecommendations(0.4, -60, 50, 0.005);
        QCOMPARE(recs1.size(), 1);  // Only quality
        
        auto recs2 = generateRecommendations(0.8, -90, 50, 0.005);
        QCOMPARE(recs2.size(), 1);  // Only RSSI
        
        auto recs3 = generateRecommendations(0.8, -60, 150, 0.005);
        QCOMPARE(recs3.size(), 1);  // Only latency
        
        auto recs4 = generateRecommendations(0.4, -90, 150, 0.05);
        QCOMPARE(recs4.size(), 4);  // All four
    }
};

QTEST_MAIN(TestMeshQoSAnalyzer)
#include "test_mesh_qos_analyzer.moc"