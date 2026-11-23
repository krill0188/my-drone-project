#ifndef POST_FLIGHT_ANALYZER_PLUGIN_H
#define POST_FLIGHT_ANALYZER_PLUGIN_H

#include <QGCPlugin.h>
#include <QObject>
#include <QString>
#include <QMap>
#include <QVector>
#include <QGeoCoordinate>
#include <QDateTime>
#include <QTimer>

class MissionManager;
class VehicleManager;
class LinkManager;
class QGCApplication;
class QmlObjectListModel;

namespace swarm::gcs {

struct FlightLogInfo {
    QString log_id;
    QString file_path;
    QString log_type;  // "ulg", "tlog", "db3"
    QDateTime flight_start;
    QDateTime flight_end;
    double duration_s = 0.0;
    uint8_t vehicle_id = 0;
    QString callsign;
    QString firmware_type;
    uint64_t file_size_bytes = 0;
    bool is_synced = false;  // Firmware + companion synced
};

struct CoverageData {
    QVector<QGeoCoordinate> flight_path;
    QVector<double> coverage_grid;  // 2D grid flattened
    double grid_resolution_m = 10.0;
    double min_lat = 0, max_lat = 0, min_lon = 0, max_lon = 0;
    int grid_width = 0, grid_height = 0;
    double total_area_m2 = 0.0;
    double covered_area_m2 = 0.0;
    double coverage_pct = 0.0;
    QMap<QString, double> sensor_coverage;  // sensor_type -> coverage %
};

struct FormationErrorData {
    QVector<double> position_errors_m;
    QVector<double> velocity_errors_mps;
    QVector<double> attitude_errors_deg;
    QVector<uint64_t> timestamps_us;
    double mean_error_m = 0.0;
    double median_error_m = 0.0;
    double std_error_m = 0.0;
    double max_error_m = 0.0;
    double p95_error_m = 0.0;
    double p99_error_m = 0.0;
    // CDF data
    QVector<double> cdf_x;  // error values
    QVector<double> cdf_y;  // cumulative probability
};

struct AITimelineEvent {
    uint64_t timestamp_us = 0;
    QString event_type;  // "detection", "classification", "replan", "fault", "landing_assessment"
    QString class_name;
    double confidence = 0.0;
    QGeoCoordinate position;
    QString details;
    uint8_t vehicle_id = 0;
    double inference_time_ms = 0.0;
};

struct QoSAnalysisData {
    QVector<double> link_quality_timeline;
    QVector<double> rssi_timeline;
    QVector<double> latency_timeline;
    QVector<double> packet_loss_timeline;
    QVector<uint64_t> timestamps_us;
    QMap<uint8_t, QVector<double>> per_link_quality;
    double avg_link_quality = 0.0;
    double min_link_quality = 0.0;
    int total_packets = 0;
    int lost_packets = 0;
    double loss_rate = 0.0;
    QVector<double> throughput_mbps;
};

struct AnalysisReport {
    QString report_id;
    QString flight_log_id;
    QDateTime generated_at;
    QString status;  // "generating", "completed", "failed"
    QString report_path;
    QString summary;
    QMap<QString, QVariant> metrics;
    QString error_message;
};

class PostFlightAnalyzerPlugin : public QGCPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.qgroundcontrol.QGCPlugin" FILE "PostFlightAnalyzer.json")
    Q_INTERFACES(QGCPlugin)

public:
    explicit PostFlightAnalyzerPlugin(QGCApplication* app, QGCToolbox* toolbox);
    ~PostFlightAnalyzerPlugin() override;

    // QGCPlugin interface
    QString name() const override { return "PostFlightAnalyzer"; }
    QString version() const override { return "1.0.0"; }
    QString description() const override { return tr("Post-flight analysis: coverage, formation CDF, AI timeline, QoS, automated reports"); }
    QString author() const override { return "AI Swarm Team"; }
    QString repository() const override { return "https://github.com/krill0188/my-drone-project"; }
    
    bool init() override;
    void start() override;
    void stop() override;
    
    QList<QGCPanelWidget*> createToolWidgets() override;
    QList<QGCPanelWidget*> createCustomWidgets() override;
    
    // Log management
    Q_INVOKABLE QVariantList listAvailableLogs() const;
    Q_INVOKABLE QVariantMap getLogInfo(const QString& log_id) const;
    Q_INVOKABLE bool loadLog(const QString& log_id);
    Q_INVOKABLE bool loadSyncedLogs(const QString& firmware_log_id, const QString& companion_log_id);
    
    // Analysis
    Q_INVOKABLE QVariantMap analyzeCoverage(const QString& log_id, const QVariantMap& params);
    Q_INVOKABLE QVariantMap analyzeFormationError(const QString& log_id);
    Q_INVOKABLE QVariantMap analyzeAITimeline(const QString& log_id);
    Q_INVOKABLE QVariantMap analyzeQoS(const QString& log_id);
    Q_INVOKABLE QVariantMap runFullAnalysis(const QString& log_id);
    
    // Report generation
    Q_INVOKABLE QString generateReport(const QString& log_id, const QVariantMap& options);
    Q_INVOKABLE QVariantMap getReportStatus(const QString& report_id) const;
    Q_INVOKABLE QVariantList listReports() const;
    Q_INVOKABLE bool openReport(const QString& report_id);
    Q_INVOKABLE bool exportReport(const QString& report_id, const QString& format);  // "pdf", "html", "json"
    
    // Python backend
    Q_INVOKABLE QVariantMap runPythonAnalysis(const QString& analysis_type, const QVariantMap& args);
    Q_INVOKABLE bool isPythonBackendAvailable() const;

signals:
    void logLoaded(const QString& log_id);
    void analysisStarted(const QString& log_id, const QString& analysis_type);
    void analysisProgress(const QString& log_id, int progress, const QString& message);
    void analysisCompleted(const QString& log_id, const QString& analysis_type, const QVariantMap& result);
    void analysisFailed(const QString& log_id, const QString& analysis_type, const QString& error);
    void reportGenerated(const QString& report_id, const QString& report_path);
    void reportGenerationProgress(const QString& report_id, int progress);
    void pythonBackendStatusChanged(bool available);
    void errorOccurred(const QString& message);

private:
    void setupConnections();
    void scanLogDirectory();
    void parseULog(const QString& file_path, FlightLogInfo& info);
    void parseTLog(const QString& file_path, FlightLogInfo& info);
    void parseRosbag(const QString& file_path, FlightLogInfo& info);
    void saveReportsIndex();
    void loadReportsIndex();
    
    // Python backend
    class PythonBackendBridge;
    PythonBackendBridge* _python_backend = nullptr;
    
    // Data
    QMap<QString, FlightLogInfo> _available_logs;
    QString _active_log_id;
    QString _log_directory;
    QString _reports_directory;
    QString _reports_index_path;
    QMap<QString, AnalysisReport> _reports;
    
    // Timers
    QTimer* _scan_timer = nullptr;
};

}  // namespace swarm::gcs

Q_DECLARE_METATYPE(swarm::gcs::FlightLogInfo)
Q_DECLARE_METATYPE(swarm::gcs::CoverageData)
Q_DECLARE_METATYPE(swarm::gcs::FormationErrorData)
Q_DECLARE_METATYPE(swarm::gcs::AITimelineEvent)
Q_DECLARE_METATYPE(swarm::gcs::QoSAnalysisData)
Q_DECLARE_METATYPE(swarm::gcs::AnalysisReport)

#endif // POST_FLIGHT_ANALYZER_PLUGIN_H