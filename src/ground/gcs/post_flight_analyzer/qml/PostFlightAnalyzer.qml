import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
import QtCharts 2.15
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.FactSystem 1.0
import QGroundControl.FlightMap 1.0
import QGroundControl.ScreenTools 1.0
import QGroundControl.Palette 1.0
import PostFlightAnalyzer 1.0

Item {
    id: root
    property PostFlightAnalyzer analyzer: null
    property var activeLogId: ""
    property var availableLogs: []
    property var coverageResult: {}
    property var formationResult: {}
    property var aiTimelineResult: {}
    property var qosResult: {}
    property var reports: []
    property var activeReportId: ""
    property var reportGenerationProgress: 0
    
    // Load available logs on startup
    Component.onCompleted: {
        if (analyzer) {
            availableLogs = analyzer.listAvailableLogs()
            reports = analyzer.listReports()
        }
    }
    
    ColumnLayout {
        anchors.fill: parent
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Toolbar
        ToolBar {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 3
            
            Label {
                text: qsTr("Flight Log:")
                Layout.alignment: Qt.AlignVCenter
            }
            
            ComboBox {
                id: logSelector
                model: root.availableLogs
                textRole: "callsign"
                valueRole: "log_id"
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 30
                onActivated: {
                    root.activeLogId = model[index].log_id
                    if (analyzer) analyzer.loadLog(root.activeLogId)
                }
            }
            
            ToolButton {
                text: qsTr("Refresh")
                iconSource: "qrc:/qmlIcons/refresh.svg"
                onClicked: {
                    if (analyzer) {
                        availableLogs = analyzer.listAvailableLogs()
                    }
                }
            }
            
            ToolButton {
                text: qsTr("Load Synced")
                enabled: logSelector.count >= 2
                onClicked: {
                    // Would open dialog to select firmware + companion logs
                }
            }
            
            Item { Layout.fillWidth: true }
            
            Label {
                id: pythonStatusLabel
                text: analyzer && analyzer.isPythonBackendAvailable() ? 
                      qsTr("Python: Connected") : qsTr("Python: Disconnected")
                color: analyzer && analyzer.isPythonBackendAvailable() ? qgcPal.color("qgc-green") : qgcPal.color("qgc-red")
                font.bold: true
            }
        }
        
        // Flight info cards
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 5
            spacing: ScreenTools.defaultFontPixelWidth * 2
            
            FlightInfoCard { title: qsTr("Flight ID"); value: activeLogInfo.vehicle_id ? "#" + activeLogInfo.vehicle_id : "—" }
            FlightInfoCard { title: qsTr("Callsign"); value: activeLogInfo.callsign || "—" }
            FlightInfoCard { title: qsTr("Duration"); value: activeLogInfo.duration_s ? (activeLogInfo.duration_s / 60).toFixed(1) + " min" : "—" }
            FlightInfoCard { title: qsTr("Type"); value: activeLogInfo.log_type || "—" }
            FlightInfoCard { title: qsTr("Size"); value: activeLogInfo.file_size_mb ? activeLogInfo.file_size_mb.toFixed(1) + " MB" : "—" }
            FlightInfoCard { title: qsTr("Synced"); value: activeLogInfo.is_synced ? qsTr("Yes") : qsTr("No"); color: activeLogInfo.is_synced ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange") }
        }
        
        // Analysis tabs
        TabBar {
            id: tabBar
            Layout.fillWidth: true
            TabButton { text: qsTr("Coverage") }
            TabButton { text: qsTr("Formation CDF") }
            TabButton { text: qsTr("AI Timeline") }
            TabButton { text: qsTr("QoS Analysis") }
            TabButton { text: qsTr("Reports") }
        }
        
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabBar.currentIndex
            
            // Coverage Tab
            CoverageHeatmapView {
                analyzer: root.analyzer
                logId: root.activeLogId
                coverageResult: root.coverageResult
                onAnalysisRequested: {
                    if (analyzer) {
                        var result = analyzer.analyzeCoverage(logId, params)
                        if (result.success) {
                            coverageResult = result.result
                        }
                    }
                }
            }
            
            // Formation CDF Tab
            FormationErrorCDFView {
                analyzer: root.analyzer
                logId: root.activeLogId
                formationResult: root.formationResult
                onAnalysisRequested: {
                    if (analyzer) {
                        var result = analyzer.analyzeFormationError(logId)
                        if (result.success) {
                            formationResult = result.result
                        }
                    }
                }
            }
            
            // AI Timeline Tab
            AITimelineView {
                analyzer: root.analyzer
                logId: root.activeLogId
                aiTimelineResult: root.aiTimelineResult
                onAnalysisRequested: {
                    if (analyzer) {
                        var result = analyzer.analyzeAITimeline(logId)
                        if (result.success) {
                            aiTimelineResult = result.result
                        }
                    }
                }
            }
            
            // QoS Analysis Tab
            QoSAnalysisView {
                analyzer: root.analyzer
                logId: root.activeLogId
                qosResult: root.qosResult
                onAnalysisRequested: {
                    if (analyzer) {
                        var result = analyzer.analyzeQoS(logId)
                        if (result.success) {
                            qosResult = result.result
                        }
                    }
                }
            }
            
            // Reports Tab
            ReportGeneratorView {
                analyzer: root.analyzer
                logId: root.activeLogId
                reports: root.reports
                onReportGenerated: {
                    reports = analyzer.listReports()
                }
                onExportRequested: {
                    if (analyzer) analyzer.exportReport(reportId, format)
                }
            }
        }
    }
    
    // Connections
    Connections {
        target: analyzer
        onLogLoaded: {
            activeLogInfo = analyzer.getLogInfo(logId)
        }
        onAnalysisProgress: {
            // Show progress in status bar
        }
        onAnalysisCompleted: {
            console.log("Analysis completed:", analysisType, result)
        }
        onAnalysisFailed: {
            console.error("Analysis failed:", analysisType, error)
        }
        onReportGenerated: {
            reports = analyzer.listReports()
        }
        onReportGenerationProgress: {
            reportGenerationProgress = progress
        }
        onPythonBackendStatusChanged: {
            pythonStatusLabel.text = available ? qsTr("Python: Connected") : qsTr("Python: Disconnected")
        }
        onErrorOccurred: {
            console.error("PostFlightAnalyzer Error:", message)
        }
    }
    
    // Read-only property for active log info
    readonly property var activeLogInfo: analyzer && activeLogId ? analyzer.getLogInfo(activeLogId) : {}
    
    // Flight info card component
    Component {
        id: FlightInfoCard
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: ScreenTools.defaultFontPixelWidth * 2
            color: qgcPal.window
            border.color: qgcPal.color("qgc-border")
            border.width: 1
            
            property string title: ""
            property string value: ""
            property color color: qgcPal.color("qgc-primary")
            
            Column {
                anchors.centerIn: parent
                spacing: ScreenTools.defaultFontPixelWidth
                
                Text {
                    text: value
                    font.pixelSize: ScreenTools.defaultFontPixelSize * 1.5
                    font.bold: true
                    color: parent.color
                    horizontalAlignment: Text.AlignHCenter
                }
                Text {
                    text: title
                    font.pixelSize: ScreenTools.defaultFontPixelSize * 0.8
                    color: qgcPal.color("qgc-text-secondary")
                    horizontalAlignment: Text.AlignHCenter
                }
            }
        }
    }
}