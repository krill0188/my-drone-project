#include "PostFlightAnalyzerPlugin.h"
#include <QGCApplication.h>
#include <QGCToolbox.h>
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QUuid>
#include <QDebug>
#include <QProcess>
#include <QEventLoop>
#include <QTimer>
#include <QDesktopServices>
#include <QUrl>
#include <algorithm>
#include <numeric>

namespace swarm::gcs {

// ============================================================================
// PythonBackendBridge Implementation
// ============================================================================

class PostFlightAnalyzerPlugin::PythonBackendBridge : public QObject
{
    Q_OBJECT
public:
    explicit PythonBackendBridge(QObject* parent = nullptr) : QObject(parent) {
        _process = new QProcess(this);
        connect(_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, &PythonBackendBridge::onProcessFinished);
        connect(_process, &QProcess::readyReadStandardOutput,
                this, &PythonBackendBridge::onReadyRead);
        connect(_process, &QProcess::errorOccurred,
                this, &PythonBackendBridge::onError);
    }
    
    ~PythonBackendBridge() {
        if (_process->state() != QProcess::NotRunning) {
            _process->terminate();
            _process->waitForFinished(1000);
        }
    }
    
    bool start() {
        QString python_path = QStringLiteral("python3");
        QString script_path = QStringLiteral(":/python/post_flight_backend.py");
        
        _process->start(python_path, {script_path, "--server"});
        bool started = _process->waitForStarted(5000);
        _available = started;
        emit availabilityChanged(_available);
        return started;
    }
    
    void stop() {
        if (_process->state() != QProcess::NotRunning) {
            _process->terminate();
            _process->waitForFinished(1000);
        }
        _available = false;
        emit availabilityChanged(false);
    }
    
    bool isAvailable() const { return _available; }
    
    QVariantMap call(const QString& method, const QVariantMap& args) {
        if (!_available) {
            return {{"error", "Python backend not available"}};
        }
        
        QVariantMap request;
        request["id"] = QUuid::createUuid().toString(QUuid::WithoutBraces);
        request["method"] = method;
        request["args"] = args;
        request["timestamp"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
        
        _pending_requests[request["id"].toString()] = QEventLoop();
        
        QJsonDocument doc(QJsonObject::fromVariantMap(request));
        _process->write(doc.toJson(QJsonDocument::Compact) + "\n");
        _process->waitForBytesWritten(1000);
        
        QEventLoop* loop = _pending_requests[request["id"].toString()];
        QTimer::singleShot(60000, loop, &QEventLoop::quit);  // Longer timeout for analysis
        loop->exec();
        
        return _responses.take(request["id"].toString());
    }

signals:
    void availabilityChanged(bool available);
    void outputReceived(const QString& line);

private slots:
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
        _available = false;
        emit availabilityChanged(false);
        qDebug() << "Python post-flight backend exited:" << exitCode << exitStatus;
    }
    
    void onReadyRead() {
        while (_process->canReadLine()) {
            QByteArray line = _process->readLine().trimmed();
            if (line.isEmpty()) continue;
            
            QJsonDocument doc = QJsonDocument::fromJson(line);
            if (doc.isObject()) {
                QVariantMap response = doc.object().toVariantMap();
                QString id = response["id"].toString();
                
                if (_pending_requests.contains(id)) {
                    _responses[id] = response;
                    _pending_requests[id].quit();
                    _pending_requests.remove(id);
                }
            }
            emit outputReceived(QString::fromUtf8(line));
        }
    }
    
    void onError(QProcess::ProcessError error) {
        qWarning() << "Python post-flight backend error:" << error;
        _available = false;
        emit availabilityChanged(false);
    }

private:
    QProcess* _process;
    bool _available = false;
    QMap<QString, QEventLoop> _pending_requests;
    QMap<QString, QVariantMap> _responses;
};

// ============================================================================
// PostFlightAnalyzerPlugin Implementation
// ============================================================================

PostFlightAnalyzerPlugin::PostFlightAnalyzerPlugin(QGCApplication* app, QGCToolbox* toolbox)
    : QGCPlugin(app, toolbox)
{
    _python_backend = new PythonBackendBridge(this);
    
    _scan_timer = new QTimer(this);
    _scan_timer->setInterval(30000);  // Scan every 30 seconds
    connect(_scan_timer, &QTimer::timeout, this, &PostFlightAnalyzerPlugin::scanLogDirectory);
    
    setupConnections();
}

PostFlightAnalyzerPlugin::~PostFlightAnalyzerPlugin() = default;

bool PostFlightAnalyzerPlugin::init() {
    _log_directory = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/logs";
    _reports_directory = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/reports";
    _reports_index_path = _reports_directory + "/reports_index.json";
    
    QDir().mkpath(_log_directory);
    QDir().mkpath(_reports_directory);
    
    scanLogDirectory();
    loadReportsIndex();
    _python_backend->start();
    _scan_timer->start();
    
    return true;
}

void PostFlightAnalyzerPlugin::start() {
    qmlRegisterType<PostFlightAnalyzerPlugin>("PostFlightAnalyzer", 1, 0, "PostFlightAnalyzer");
}

void PostFlightAnalyzerPlugin::stop() {
    _scan_timer->stop();
    _python_backend->stop();
    saveReportsIndex();
}

QList<QGCPanelWidget*> PostFlightAnalyzerPlugin::createToolWidgets() {
    return QList<QGCPanelWidget*>();
}

QList<QGCPanelWidget*> PostFlightAnalyzerPlugin::createCustomWidgets() {
    return QList<QGCPanelWidget*>();
}

void PostFlightAnalyzerPlugin::setupConnections() {
    connect(_python_backend, &PythonBackendBridge::availabilityChanged,
            this, &PostFlightAnalyzerPlugin::pythonBackendStatusChanged);
}

void PostFlightAnalyzerPlugin::scanLogDirectory() {
    QDir dir(_log_directory);
    QStringList filters = {"*.ulg", "*.tlog", "*.db3", "*.log"};
    QStringList files = dir.entryList(filters, QDir::Files | QDir::Time, QDir::Reversed);
    
    for (const QString& file : files) {
        QString file_path = dir.absoluteFilePath(file);
        QString log_id = QCryptographicHash::hash(file_path.toUtf8(), QCryptographicHash::Md5).toHex().left(16);
        
        if (!_available_logs.contains(log_id)) {
            FlightLogInfo info;
            info.log_id = log_id;
            info.file_path = file_path;
            info.file_size_bytes = QFileInfo(file_path).size();
            
            if (file.endsWith(".ulg", Qt::CaseInsensitive)) {
                info.log_type = "ulg";
                parseULog(file_path, info);
            } else if (file.endsWith(".tlog", Qt::CaseInsensitive)) {
                info.log_type = "tlog";
                parseTLog(file_path, info);
            } else if (file.endsWith(".db3", Qt::CaseInsensitive)) {
                info.log_type = "rosbag2";
                parseRosbag(file_path, info);
            }
            
            _available_logs[log_id] = info;
        }
    }
}

void PostFlightAnalyzerPlugin::parseULog(const QString& file_path, FlightLogInfo& info) {
    // Would use pyulog or similar to parse ULog
    // For now, extract basic info from filename
    QFileInfo fi(file_path);
    info.flight_start = fi.birthTime();
    info.flight_end = fi.lastModified();
    info.duration_s = info.flight_start.secsTo(info.flight_end);
}

void PostFlightAnalyzerPlugin::parseTLog(const QString& file_path, FlightLogInfo& info) {
    QFileInfo fi(file_path);
    info.flight_start = fi.birthTime();
    info.flight_end = fi.lastModified();
    info.duration_s = info.flight_start.secsTo(info.flight_end);
}

void PostFlightAnalyzerPlugin::parseRosbag(const QString& file_path, FlightLogInfo& info) {
    QFileInfo fi(file_path);
    info.flight_start = fi.birthTime();
    info.flight_end = fi.lastModified();
    info.duration_s = info.flight_start.secsTo(info.flight_end);
    info.is_synced = true;  // Rosbag2 typically has companion data
}

QVariantList PostFlightAnalyzerPlugin::listAvailableLogs() const {
    QVariantList list;
    for (auto it = _available_logs.constBegin(); it != _available_logs.constEnd(); ++it) {
        const FlightLogInfo& info = it.value();
        QVariantMap map;
        map["log_id"] = info.log_id;
        map["file_path"] = info.file_path;
        map["log_type"] = info.log_type;
        map["flight_start"] = info.flight_start.toString(Qt::ISODate);
        map["flight_end"] = info.flight_end.toString(Qt::ISODate);
        map["duration_s"] = info.duration_s;
        map["vehicle_id"] = info.vehicle_id;
        map["callsign"] = info.callsign;
        map["firmware_type"] = info.firmware_type;
        map["file_size_mb"] = info.file_size_bytes / (1024.0 * 1024.0);
        map["is_synced"] = info.is_synced;
        list.append(map);
    }
    return list;
}

QVariantMap PostFlightAnalyzerPlugin::getLogInfo(const QString& log_id) const {
    QVariantMap map;
    if (!_available_logs.contains(log_id)) return map;
    
    const FlightLogInfo& info = _available_logs[log_id];
    map["log_id"] = info.log_id;
    map["file_path"] = info.file_path;
    map["log_type"] = info.log_type;
    map["flight_start"] = info.flight_start.toString(Qt::ISODate);
    map["flight_end"] = info.flight_end.toString(Qt::ISODate);
    map["duration_s"] = info.duration_s;
    map["vehicle_id"] = info.vehicle_id;
    map["callsign"] = info.callsign;
    map["firmware_type"] = info.firmware_type;
    map["file_size_mb"] = info.file_size_bytes / (1024.0 * 1024.0);
    map["is_synced"] = info.is_synced;
    
    return map;
}

bool PostFlightAnalyzerPlugin::loadLog(const QString& log_id) {
    if (!_available_logs.contains(log_id)) return false;
    
    _active_log_id = log_id;
    emit logLoaded(log_id);
    return true;
}

bool PostFlightAnalyzerPlugin::loadSyncedLogs(const QString& firmware_log_id, const QString& companion_log_id) {
    if (!_available_logs.contains(firmware_log_id) || !_available_logs.contains(companion_log_id)) {
        return false;
    }
    
    // Would run log synchronization here
    _active_log_id = firmware_log_id + "_" + companion_log_id;
    emit logLoaded(_active_log_id);
    return true;
}

QVariantMap PostFlightAnalyzerPlugin::analyzeCoverage(const QString& log_id, const QVariantMap& params) {
    if (!_available_logs.contains(log_id)) {
        return {{"error", "Log not found"}};
    }
    
    emit analysisStarted(log_id, "coverage");
    
    QVariantMap args = params;
    args["log_id"] = log_id;
    args["log_path"] = _available_logs[log_id].file_path;
    
    QVariantMap result = _python_backend->call("analyze_coverage", args);
    
    if (result["success"].toBool()) {
        emit analysisCompleted(log_id, "coverage", result["result"].toMap());
    } else {
        emit analysisFailed(log_id, "coverage", result["error"].toString());
    }
    
    return result;
}

QVariantMap PostFlightAnalyzerPlugin::analyzeFormationError(const QString& log_id) {
    if (!_available_logs.contains(log_id)) {
        return {{"error", "Log not found"}};
    }
    
    emit analysisStarted(log_id, "formation_error");
    
    QVariantMap args;
    args["log_id"] = log_id;
    args["log_path"] = _available_logs[log_id].file_path;
    
    QVariantMap result = _python_backend->call("analyze_formation_error", args);
    
    if (result["success"].toBool()) {
        emit analysisCompleted(log_id, "formation_error", result["result"].toMap());
    } else {
        emit analysisFailed(log_id, "formation_error", result["error"].toString());
    }
    
    return result;
}

QVariantMap PostFlightAnalyzerPlugin::analyzeAITimeline(const QString& log_id) {
    if (!_available_logs.contains(log_id)) {
        return {{"error", "Log not found"}};
    }
    
    emit analysisStarted(log_id, "ai_timeline");
    
    QVariantMap args;
    args["log_id"] = log_id;
    args["log_path"] = _available_logs[log_id].file_path;
    
    QVariantMap result = _python_backend->call("analyze_ai_timeline", args);
    
    if (result["success"].toBool()) {
        emit analysisCompleted(log_id, "ai_timeline", result["result"].toMap());
    } else {
        emit analysisFailed(log_id, "ai_timeline", result["error"].toString());
    }
    
    return result;
}

QVariantMap PostFlightAnalyzerPlugin::analyzeQoS(const QString& log_id) {
    if (!_available_logs.contains(log_id)) {
        return {{"error", "Log not found"}};
    }
    
    emit analysisStarted(log_id, "qos");
    
    QVariantMap args;
    args["log_id"] = log_id;
    args["log_path"] = _available_logs[log_id].file_path;
    
    QVariantMap result = _python_backend->call("analyze_qos", args);
    
    if (result["success"].toBool()) {
        emit analysisCompleted(log_id, "qos", result["result"].toMap());
    } else {
        emit analysisFailed(log_id, "qos", result["error"].toString());
    }
    
    return result;
}

QVariantMap PostFlightAnalyzerPlugin::runFullAnalysis(const QString& log_id) {
    if (!_available_logs.contains(log_id)) {
        return {{"error", "Log not found"}};
    }
    
    emit analysisStarted(log_id, "full");
    
    QVariantMap args;
    args["log_id"] = log_id;
    args["log_path"] = _available_logs[log_id].file_path;
    
    QVariantMap result = _python_backend->call("run_full_analysis", args);
    
    if (result["success"].toBool()) {
        emit analysisCompleted(log_id, "full", result["result"].toMap());
    } else {
        emit analysisFailed(log_id, "full", result["error"].toString());
    }
    
    return result;
}

QString PostFlightAnalyzerPlugin::generateReport(const QString& log_id, const QVariantMap& options) {
    if (!_available_logs.contains(log_id)) {
        return "";
    }
    
    QString report_id = QUuid::createUuid().toString(QUuid::WithoutBraces);
    
    AnalysisReport report;
    report.report_id = report_id;
    report.flight_log_id = log_id;
    report.generated_at = QDateTime::currentDateTimeUtc();
    report.status = "generating";
    
    _reports[report_id] = report;
    
    // Run report generation asynchronously via Python backend
    QVariantMap args = options;
    args["log_id"] = log_id;
    args["log_path"] = _available_logs[log_id].file_path;
    args["report_id"] = report_id;
    args["output_dir"] = _reports_directory;
    args["format"] = options.value("format", "html").toString();
    
    // Start generation (non-blocking)
    QTimer::singleShot(0, [this, report_id, args]() {
        QVariantMap result = _python_backend->call("generate_report", args);
        
        if (result["success"].toBool()) {
            _reports[report_id].status = "completed";
            _reports[report_id].report_path = result["result"].toMap().value("report_path").toString();
            _reports[report_id].summary = result["result"].toMap().value("summary").toString();
            emit reportGenerated(report_id, _reports[report_id].report_path);
        } else {
            _reports[report_id].status = "failed";
            _reports[report_id].error_message = result["error"].toString();
        }
        saveReportsIndex();
    });
    
    return report_id;
}

QVariantMap PostFlightAnalyzerPlugin::getReportStatus(const QString& report_id) const {
    QVariantMap map;
    if (!_reports.contains(report_id)) return map;
    
    const AnalysisReport& r = _reports[report_id];
    map["report_id"] = r.report_id;
    map["flight_log_id"] = r.flight_log_id;
    map["generated_at"] = r.generated_at.toString(Qt::ISODate);
    map["status"] = r.status;
    map["report_path"] = r.report_path;
    map["summary"] = r.summary;
    map["error_message"] = r.error_message;
    
    return map;
}

QVariantList PostFlightAnalyzerPlugin::listReports() const {
    QVariantList list;
    for (auto it = _reports.constBegin(); it != _reports.constEnd(); ++it) {
        const AnalysisReport& r = it.value();
        QVariantMap map;
        map["report_id"] = r.report_id;
        map["flight_log_id"] = r.flight_log_id;
        map["generated_at"] = r.generated_at.toString(Qt::ISODate);
        map["status"] = r.status;
        map["report_path"] = r.report_path;
        map["summary"] = r.summary;
        list.append(map);
    }
    return list;
}

bool PostFlightAnalyzerPlugin::openReport(const QString& report_id) {
    if (!_reports.contains(report_id)) return false;
    
    const AnalysisReport& r = _reports[report_id];
    if (r.status != "completed" || r.report_path.isEmpty()) return false;
    
    return QDesktopServices::openUrl(QUrl::fromLocalFile(r.report_path));
}

bool PostFlightAnalyzerPlugin::exportReport(const QString& report_id, const QString& format) {
    if (!_reports.contains(report_id)) return false;
    
    const AnalysisReport& r = _reports[report_id];
    if (r.status != "completed" || r.report_path.isEmpty()) return false;
    
    // Would convert report to different format
    QVariantMap args;
    args["report_path"] = r.report_path;
    args["format"] = format;
    args["output_path"] = _reports_directory + "/" + report_id + "_export." + format;
    
    QVariantMap result = _python_backend->call("export_report", args);
    return result["success"].toBool();
}

QVariantMap PostFlightAnalyzerPlugin::runPythonAnalysis(const QString& analysis_type, const QVariantMap& args) {
    return _python_backend->call(analysis_type, args);
}

bool PostFlightAnalyzerPlugin::isPythonBackendAvailable() const {
    return _python_backend->isAvailable();
}

void PostFlightAnalyzerPlugin::saveReportsIndex() {
    QJsonArray arr;
    for (auto it = _reports.constBegin(); it != _reports.constEnd(); ++it) {
        const AnalysisReport& r = it.value();
        QJsonObject obj;
        obj["report_id"] = r.report_id;
        obj["flight_log_id"] = r.flight_log_id;
        obj["generated_at"] = r.generated_at.toString(Qt::ISODate);
        obj["status"] = r.status;
        obj["report_path"] = r.report_path;
        obj["summary"] = r.summary;
        obj["error_message"] = r.error_message;
        arr.append(obj);
    }
    
    QJsonDocument doc(arr);
    QFile f(_reports_index_path);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(doc.toJson(QJsonDocument::Indented));
    }
}

void PostFlightAnalyzerPlugin::loadReportsIndex() {
    QFile f(_reports_index_path);
    if (!f.open(QIODevice::ReadOnly)) return;
    
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isArray()) return;
    
    QJsonArray arr = doc.array();
    for (const auto& val : arr) {
        QJsonObject obj = val.toObject();
        AnalysisReport r;
        r.report_id = obj["report_id"].toString();
        r.flight_log_id = obj["flight_log_id"].toString();
        r.generated_at = QDateTime::fromString(obj["generated_at"].toString(), Qt::ISODate);
        r.status = obj["status"].toString();
        r.report_path = obj["report_path"].toString();
        r.summary = obj["summary"].toString();
        r.error_message = obj["error_message"].toString();
        _reports[r.report_id] = r;
    }
}

}  // namespace swarm::gcs

#include "PostFlightAnalyzerPlugin.moc"