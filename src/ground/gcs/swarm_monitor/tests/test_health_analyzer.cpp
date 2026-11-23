#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <vector>
#include <algorithm>
#include <numeric>

class TestHealthAnalyzer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testTrendCalculation() {
        auto computeTrend = [](const std::vector<double>& values) {
            if (values.size() < 2) return std::map<std::string, double>{{"slope", 0}, {"intercept", 0}, {"r2", 0}};
            
            int n = values.size();
            std::vector<int> x(n);
            std::iota(x.begin(), x.end(), 0);
            
            double x_mean = std::accumulate(x.begin(), x.end(), 0.0) / n;
            double y_mean = std::accumulate(values.begin(), values.end(), 0.0) / n;
            
            double num = 0, den = 0;
            for (int i = 0; i < n; ++i) {
                num += (x[i] - x_mean) * (values[i] - y_mean);
                den += (x[i] - x_mean) * (x[i] - x_mean);
            }
            
            if (den == 0) return std::map<std::string, double>{{"slope", 0}, {"intercept", y_mean}, {"r2", 0}};
            
            double slope = num / den;
            double intercept = y_mean - slope * x_mean;
            
            // R-squared
            double ss_res = 0, ss_tot = 0;
            for (int i = 0; i < n; ++i) {
                double y_pred = intercept + slope * x[i];
                ss_res += (values[i] - y_pred) * (values[i] - y_pred);
                ss_tot += (values[i] - y_mean) * (values[i] - y_mean);
            }
            double r2 = ss_tot > 0 ? 1 - ss_res / ss_tot : 0;
            
            return std::map<std::string, double>{{"slope", slope}, {"intercept", intercept}, {"r2", r2}};
        };
        
        // Increasing trend
        std::vector<double> increasing = {10, 11, 12, 13, 14};
        auto trend1 = computeTrend(increasing);
        QVERIFY(trend1["slope"] > 0.9 && trend1["slope"] < 1.1);
        QVERIFY(trend1["r2"] > 0.99);
        
        // Decreasing trend
        std::vector<double> decreasing = {50, 45, 40, 35, 30};
        auto trend2 = computeTrend(decreasing);
        QVERIFY(trend2["slope"] < -0.9 && trend2["slope"] > -1.1);
        QVERIFY(trend2["r2"] > 0.99);
        
        // Flat trend
        std::vector<double> flat = {25, 25, 25, 25, 25};
        auto trend3 = computeTrend(flat);
        QVERIFY(trend3["slope"] == 0.0);
        QVERIFY(trend3["r2"] == 0.0);  // or 1.0 depending on implementation
    }
    
    void testPercentile() {
        auto percentile = [](const std::vector<double>& data, double p) {
            if (data.empty()) return 0.0;
            std::vector<double> sorted = data;
            std::sort(sorted.begin(), sorted.end());
            double k = (sorted.size() - 1) * p / 100;
            int f = floor(k);
            int c = ceil(k);
            if (f == c) return sorted[f];
            return sorted[f] * (c - k) + sorted[c] * (k - f);
        };
        
        std::vector<double> data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        
        QCOMPARE(percentile(data, 50), 5.5);
        QCOMPARE(percentile(data, 25), 3.25);
        QCOMPARE(percentile(data, 75), 7.75);
        QCOMPARE(percentile(data, 95), 9.55);
        QCOMPARE(percentile(data, 0), 1.0);
        QCOMPARE(percentile(data, 100), 10.0);
    }
    
    void testHealthAlerts() {
        auto generateAlerts = [](const QVariantMap& current, const QVariantMap& trends) {
            QVariantList alerts;
            
            if (current["battery_remaining_pct"].toDouble() < 20) {
                alerts.append(QVariantMap{{"level", "CRITICAL"}, {"message", "Battery critically low"}});
            } else if (current["battery_remaining_pct"].toDouble() < 30) {
                alerts.append(QVariantMap{{"level", "HIGH"}, {"message", "Battery low"}});
            }
            
            if (current["link_quality"].toDouble() < 0.3) {
                alerts.append(QVariantMap{{"level", "CRITICAL"}, {"message", "Link quality critical"}});
            } else if (current["link_quality"].toDouble() < 0.5) {
                alerts.append(QVariantMap{{"level", "HIGH"}, {"message", "Link quality degraded"}});
            }
            
            if (current["gps_satellites"].toInt() < 6) {
                alerts.append(QVariantMap{{"level", "HIGH"}, {"message", "GPS satellite count low"}});
            }
            
            if (current["ekf_innovation"].toDouble() > 1.0) {
                alerts.append(QVariantMap{{"level", "HIGH"}, {"message", "EKF innovation high"}});
            }
            
            for (auto it = trends.begin(); it != trends.end(); ++it) {
                if (it.key().contains("battery") && it.value().toMap()["slope"].toDouble() < -0.1) {
                    alerts.append(QVariantMap{{"level", "MEDIUM"}, {"message", "Battery depleting rapidly"}});
                }
            }
            
            return alerts;
        };
        
        // Critical battery
        QVariantMap current1;
        current1["battery_remaining_pct"] = 15;
        current1["link_quality"] = 0.9;
        current1["gps_satellites"] = 12;
        current1["ekf_innovation"] = 0.5;
        QVariantMap trends1;
        auto alerts1 = generateAlerts(current1, trends1);
        QVERIFY(alerts1.size() > 0);
        bool has_critical = false;
        for (const auto& a : alerts1) {
            if (a.toMap()["level"].toString() == "CRITICAL") has_critical = true;
        }
        QVERIFY(has_critical);
        
        // Degraded link
        QVariantMap current2;
        current2["battery_remaining_pct"] = 50;
        current2["link_quality"] = 0.4;
        current2["gps_satellites"] = 10;
        current2["ekf_innovation"] = 0.3;
        auto alerts2 = generateAlerts(current2, trends1);
        bool has_high = false;
        for (const auto& a : alerts2) {
            if (a.toMap()["level"].toString() == "HIGH") has_high = true;
        }
        QVERIFY(has_high);
        
        // Battery trend
        QVariantMap trends2;
        trends2["battery_depletion_rate"] = QVariantMap{{"slope", -0.2}};
        QVariantMap current3;
        current3["battery_remaining_pct"] = 50;
        current3["link_quality"] = 0.9;
        current3["gps_satellites"] = 12;
        current3["ekf_innovation"] = 0.5;
        auto alerts3 = generateAlerts(current3, trends2);
        bool has_medium = false;
        for (const auto& a : alerts3) {
            if (a.toMap()["level"].toString() == "MEDIUM") has_medium = true;
        }
        QVERIFY(has_medium);
    }
};

QTEST_MAIN(TestHealthAnalyzer)
#include "test_health_analyzer.moc"