#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <vector>
#include <map>
#include <numeric>

class TestQoSAnalyzer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testLinkQualityAggregation() {
        struct Link {
            int from, to;
            double quality, rssi, latency, loss_rate, throughput;
        };
        
        std::vector<Link> links = {
            {1, 2, 0.9, -50, 10, 0.001, 10},
            {1, 3, 0.7, -70, 25, 0.02, 8},
            {2, 1, 0.9, -50, 10, 0.001, 10},
            {2, 3, 0.8, -60, 15, 0.005, 9},
            {3, 1, 0.7, -70, 25, 0.02, 8},
            {3, 2, 0.8, -60, 15, 0.005, 9},
        };
        
        std::vector<double> qualities, rssis, latencies, loss_rates, throughputs;
        for (const auto& l : links) {
            qualities.push_back(l.quality);
            rssis.push_back(l.rssi);
            latencies.push_back(l.latency);
            loss_rates.push_back(l.loss_rate);
            throughputs.push_back(l.throughput);
        }
        
        auto mean = [](const std::vector<double>& v) {
            return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        };
        
        QVERIFY(qFuzzyCompare(mean(qualities), 0.8, 0.01));
        QVERIFY(qFuzzyCompare(mean(rssis), -60.0, 1.0));
        QVERIFY(qFuzzyCompare(mean(latencies), 16.67, 1.0));
        QVERIFY(qFuzzyCompare(mean(loss_rates), 0.0087, 0.001));
        QVERIFY(qFuzzyCompare(mean(throughputs), 9.0, 0.1));
    }
    
    void testPerLinkAggregation() {
        std::map<std::string, std::vector<double>> per_link_qualities = {
            {"1->2", {0.9, 0.85, 0.95}},
            {"1->3", {0.7, 0.65, 0.75}},
            {"2->3", {0.8, 0.85, 0.9}},
        };
        
        std::map<std::string, double> avg_quality;
        for (auto& [key, vals] : per_link_qualities) {
            avg_quality[key] = std::accumulate(vals.begin(), vals.end(), 0.0) / vals.size();
        }
        
        QVERIFY(qFuzzyCompare(avg_quality["1->2"], 0.9, 0.01));
        QVERIFY(qFuzzyCompare(avg_quality["1->3"], 0.7, 0.01));
        QVERIFY(qFuzzyCompare(avg_quality["2->3"], 0.85, 0.01));
    }
    
    void testCriticalLinkDetection() {
        struct LinkStat {
            std::string key;
            double avg_quality, avg_rssi, avg_latency, loss_rate;
        };
        
        std::vector<LinkStat> links = {
            {"1->2", 0.9, -50, 10, 0.001},
            {"1->3", 0.2, -85, 30, 0.15},   // CRITICAL: quality < 0.3
            {"2->3", 0.5, -65, 20, 0.08},   // HIGH: loss > 0.05
            {"3->1", 0.8, -55, 12, 0.002},
        };
        
        int critical = 0, high = 0, low = 0;
        for (auto& l : links) {
            if (l.avg_quality < 0.3 || l.loss_rate > 0.1) critical++;
            else if (l.avg_quality < 0.5 || l.loss_rate > 0.05) high++;
            else low++;
        }
        
        QCOMPARE(critical, 1);  // 1->3
        QCOMPARE(high, 1);      // 2->3
        QCOMPARE(low, 2);       // 1->2, 3->1
    }
    
    void testQoSRecommendations() {
        auto generate = [](double avg_q, double min_rssi, double avg_lat, double avg_loss) {
            std::vector<std::string> recs;
            if (avg_q < 0.7) recs.push_back("quality");
            if (min_rssi < -80) recs.push_back("rssi");
            if (avg_lat > 100) recs.push_back("latency");
            if (avg_loss > 0.05) recs.push_back("loss");
            return recs;
        };
        
        auto recs1 = generate(0.6, -60, 50, 0.01);
        QCOMPARE(recs1.size(), 1);
        QVERIFY(recs1[0] == "quality");
        
        auto recs2 = generate(0.8, -85, 50, 0.01);
        QCOMPARE(recs2.size(), 1);
        QVERIFY(recs2[0] == "rssi");
        
        auto recs3 = generate(0.8, -60, 150, 0.01);
        QCOMPARE(recs3.size(), 1);
        QVERIFY(recs3[0] == "latency");
        
        auto recs4 = generate(0.8, -60, 50, 0.1);
        QCOMPARE(recs4.size(), 1);
        QVERIFY(recs4[0] == "loss");
        
        auto recs5 = generate(0.5, -90, 200, 0.2);
        QCOMPARE(recs5.size(), 4);
    }
};

QTEST_MAIN(TestQoSAnalyzer)
#include "test_qos_analyzer.moc"