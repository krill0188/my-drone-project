import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.ScreenTools 1.0
import QGroundControl.Palette 1.0

Item {
    id: root
    property PostFlightAnalyzer analyzer: null
    property string logId: ""
    property var reports: []
    
    signal reportGenerated()
    signal exportRequested(string reportId, string format)
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Report generation panel
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 12
            title: qsTr("Generate Report")
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                spacing: ScreenTools.defaultFontPixelWidth * 2
                
                RowLayout {
                    Label { text: qsTr("Flight Log:"); Layout.alignment: Qt.AlignVCenter }
                    ComboBox {
                        id: reportLogSelector
                        model: analyzer ? analyzer.listAvailableLogs() : []
                        textRole: "callsign"
                        valueRole: "log_id"
                        Layout.fillWidth: true
                    }
                }
                
                RowLayout {
                    Label { text: qsTr("Format:"); Layout.alignment: Qt.AlignVCenter }
                    ComboBox {
                        id: formatSelector
                        model: ["HTML", "PDF", "JSON"]
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 15
                    }
                }
                
                RowLayout {
                    Label { text: qsTr("Include:"); Layout.alignment: Qt.AlignVCenter }
                    CheckBox { text: qsTr("Coverage Heatmap"); id: includeCoverage; checked: true; Layout.alignment: Qt.AlignVCenter }
                    CheckBox { text: qsTr("Formation CDF"); id: includeFormation; checked: true; Layout.alignment: Qt.AlignVCenter }
                    CheckBox { text: qsTr("AI Timeline"); id: includeAI; checked: true; Layout.alignment: Qt.AlignVCenter }
                    CheckBox { text: qsTr("QoS Analysis"); id: includeQoS; checked: true; Layout.alignment: Qt.AlignVCenter }
                }
                
                RowLayout {
                    Label { text: qsTr("Template:"); Layout.alignment: Qt.AlignVCenter }
                    ComboBox {
                        id: templateSelector
                        model: ["Standard", "Detailed", "Executive Summary", "Regulatory"]
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 20
                    }
                }
                
                RowLayout {
                    ToolButton {
                        text: qsTr("Generate Report")
                        Layout.fillWidth: true
                        enabled: reportLogSelector.count > 0
                        onClicked: {
                            if (analyzer) {
                                var options = {
                                    "format": formatSelector.currentText.toLowerCase(),
                                    "include_coverage": includeCoverage.checked,
                                    "include_formation": includeFormation.checked,
                                    "include_ai": includeAI.checked,
                                    "include_qos": includeQoS.checked,
                                    "template": templateSelector.currentText.toLowerCase().replace(" ", "_")
                                }
                                var reportId = analyzer.generateReport(reportLogSelector.currentValue, options)
                                if (reportId) {
                                    activeReportId = reportId
                                }
                            }
                        }
                    }
                }
                
                ProgressBar {
                    Layout.fillWidth: true
                    visible: reportGenerationProgress > 0 && reportGenerationProgress < 100
                    from: 0
                    to: 100
                    value: reportGenerationProgress
                }
            }
        }
        
        // Reports list
        GroupBox {
            Layout.fillWidth: true
            Layout.fillHeight: true
            title: qsTr("Generated Reports (%1)").arg(reports.length)
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.reports
                sortIndicatorVisible: true
                
                TableViewColumn {
                    role: "report_id"
                    title: qsTr("Report ID")
                    width: ScreenTools.defaultFontPixelWidth * 20
                }
                TableViewColumn {
                    role: "flight_log_id"
                    title: qsTr("Flight Log")
                    width: ScreenTools.defaultFontPixelWidth * 20
                }
                TableViewColumn {
                    role: "generated_at"
                    title: qsTr("Generated")
                    width: ScreenTools.defaultFontPixelWidth * 20
                    delegate: ItemDelegate { 
                        text: Qt.formatDateTime(new Date(display), "yyyy-MM-dd HH:mm")
                    }
                }
                TableViewColumn {
                    role: "status"
                    title: qsTr("Status")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: display
                        color: display === "completed" ? qgcPal.color("qgc-green") : 
                               display === "generating" ? qgcPal.color("qgc-blue") : qgcPal.color("qgc-red")
                        font.bold: display === "completed"
                    }
                }
                TableViewColumn {
                    role: "report_path"
                    title: qsTr("Path")
                    width: ScreenTools.defaultFontPixelWidth * 30
                }
                TableViewColumn {
                    role: ""
                    title: qsTr("Actions")
                    width: ScreenTools.defaultFontPixelWidth * 30
                    delegate: ItemDelegate {
                        RowLayout {
                            anchors.centerIn: parent
                            spacing: ScreenTools.defaultFontPixelWidth
                            
                            ToolButton {
                                text: qsTr("Open")
                                enabled: display.status === "completed"
                                onClicked: {
                                    if (analyzer) analyzer.openReport(display.report_id)
                                }
                            }
                            ToolButton {
                                text: qsTr("Export")
                                enabled: display.status === "completed"
                                onClicked: {
                                    exportDialog.property("selectedReportId", display.report_id)
                                    exportDialog.open()
                                }
                            }
                            ToolButton {
                                text: qsTr("Delete")
                                onClicked: {
                                    // Would delete report
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Export dialog
    Dialog {
        id: exportDialog
        title: qsTr("Export Report")
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel
        property string selectedReportId: ""
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: ScreenTools.defaultFontPixelWidth * 2
            spacing: ScreenTools.defaultFontPixelWidth * 2
            
            Label { text: qsTr("Export format:") }
            ComboBox {
                id: exportFormat
                model: ["PDF", "HTML", "JSON", "CSV (metrics only)"]
                Layout.fillWidth: true
            }
            
            Label { text: qsTr("Include sections:") }
            CheckBox { text: qsTr("Coverage"); checked: true }
            CheckBox { text: qsTr("Formation CDF"); checked: true }
            CheckBox { text: qsTr("AI Timeline"); checked: true }
            CheckBox { text: qsTr("QoS"); checked: true }
        }
        
        onAccepted: {
            if (analyzer && selectedReportId) {
                analyzer.exportReport(selectedReportId, exportFormat.currentText.toLowerCase())
                exportRequested(selectedReportId, exportFormat.currentText.toLowerCase())
            }
        }
    }
    
    property string activeReportId: ""
    property int reportGenerationProgress: 0
    
    // Refresh reports when model changes
    function refreshReports() {
        if (analyzer) {
            reports = analyzer.listReports()
        }
    }
    
    Connections {
        target: analyzer
        onReportGenerated: {
            refreshReports()
        }
        onReportGenerationProgress: {
            reportGenerationProgress = progress
        }
    }
    
    Component.onCompleted: {
        refreshReports()
    }
}