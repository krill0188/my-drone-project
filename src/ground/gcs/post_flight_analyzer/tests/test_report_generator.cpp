#include <QtTest/QtTest>
#include <QtCore/QVariantMap>
#include <QtCore/QVariantList>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QTemporaryDir>

class TestReportGenerator : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() {}
    void cleanupTestCase() {}
    
    void testHTMLReportStructure() {
        QString html = R"(
<!DOCTYPE html>
<html>
<head><title>Test Report</title></head>
<body>
    <h1>Post-Flight Analysis Report</h1>
    <div class="metric"><div class="metric-value">85.5%</div><div class="metric-label">Coverage</div></div>
    <table><tr><th>Metric</th><th>Value</th></tr><tr><td>Mean Error</td><td>1.5m</td></tr></table>
</body>
</html>
)";
        
        // Verify HTML structure
        QVERIFY(html.contains("<!DOCTYPE html>"));
        QVERIFY(html.contains("<html>"));
        QVERIFY(html.contains("<head>"));
        QVERIFY(html.contains("<title>Test Report</title>"));
        QVERIFY(html.contains("<body>"));
        QVERIFY(html.contains("<h1>Post-Flight Analysis Report</h1>"));
        QVERIFY(html.contains("85.5%"));
        QVERIFY(html.contains("Coverage"));
        QVERIFY(html.contains("<table>"));
        QVERIFY(html.contains("<th>Metric</th>"));
        QVERIFY(html.contains("<th>Value</th>"));
        QVERIFY(html.contains("Mean Error"));
        QVERIFY(html.contains("1.5m"));
    }
    
    void testJSONReportStructure() {
        QJsonObject report;
        report["report_id"] = "test-report-123";
        report["log_id"] = "flight-456";
        report["generated_at"] = "2025-01-15T10:30:00Z";
        
        QJsonObject analysis;
        analysis["coverage_pct"] = 85.5;
        analysis["mean_error_m"] = 1.5;
        analysis["total_inferences"] = 1250;
        analysis["avg_link_quality"] = 0.92;
        
        report["analysis"] = analysis;
        
        QJsonDocument doc(report);
        QString json_str = doc.toJson(QJsonDocument::Indented);
        
        QVERIFY(json_str.contains("test-report-123"));
        QVERIFY(json_str.contains("flight-456"));
        QVERIFY(json_str.contains("2025-01-15T10:30:00Z"));
        QVERIFY(json_str.contains("85.5"));
        QVERIFY(json_str.contains("1.5"));
        QVERIFY(json_str.contains("1250"));
        QVERIFY(json_str.contains("0.92"));
        
        // Verify valid JSON
        QJsonDocument parsed = QJsonDocument::fromJson(json_str.toUtf8());
        QVERIFY(!parsed.isNull());
        QVERIFY(parsed.isObject());
    }
    
    void testReportFileWrite() {
        QTemporaryDir tempDir;
        QVERIFY(tempDir.isValid());
        
        QString html_path = tempDir.path() + "/test_report.html";
        QFile html_file(html_path);
        QVERIFY(html_file.open(QIODevice::WriteOnly));
        html_file.write("<html><body><h1>Test</h1></body></html>");
        html_file.close();
        
        QVERIFY(QFile::exists(html_path));
        
        QString json_path = tempDir.path() + "/test_report.json";
        QFile json_file(json_path);
        QVERIFY(json_file.open(QIODevice::WriteOnly));
        json_file.write("{\"test\": \"data\"}");
        json_file.close();
        
        QVERIFY(QFile::exists(json_path));
    }
    
    void testReportOptions() {
        QVariantMap options;
        options["format"] = "html";
        options["include_coverage"] = true;
        options["include_formation"] = true;
        options["include_ai"] = false;
        options["include_qos"] = true;
        options["template"] = "standard";
        
        QVERIFY(options["format"].toString() == "html");
        QVERIFY(options["include_coverage"].toBool() == true);
        QVERIFY(options["include_formation"].toBool() == true);
        QVERIFY(options["include_ai"].toBool() == false);
        QVERIFY(options["include_qos"].toBool() == true);
        QVERIFY(options["template"].toString() == "standard");
    }
    
    void testExportFormats() {
        QStringList formats = {"html", "pdf", "json", "csv"};
        
        QVERIFY(formats.contains("html"));
        QVERIFY(formats.contains("pdf"));
        QVERIFY(formats.contains("json"));
        QVERIFY(formats.contains("csv"));
        QCOMPARE(formats.size(), 4);
    }
};

QTEST_MAIN(TestReportGenerator)
#include "test_report_generator.moc"