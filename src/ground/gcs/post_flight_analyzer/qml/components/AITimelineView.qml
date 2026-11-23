import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtCharts 2.15
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.ScreenTools 1.0
import QGroundControl.Palette 1.0

Item {
    id: root
    property PostFlightAnalyzer analyzer: null
    property string logId: ""
    property var aiTimelineResult: {}
    
    signal analysisRequested(string logId)
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Control panel
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 8
            title: qsTr("AI Inference Timeline")
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                spacing: ScreenTools.defaultFontPixelWidth * 2
                
                ToolButton {
                    text: qsTr("Analyze AI Timeline")
                    Layout.fillWidth: true
                    enabled: root.logId !== ""
                    onClicked: root.analysisRequested(root.logId)
                }
            }
        }
        
        // Summary cards
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 5
            spacing: ScreenTools.defaultFontPixelWidth * 2
            
            AIStatCard { 
                label: qsTr("Total Inferences"); 
                value: root.aiTimelineResult.total_inferences || "—" 
            }
            AIStatCard { 
                label: qsTr("Avg Inference Time"); 
                value: root.aiTimelineResult.avg_inference_time_ms ? root.aiTimelineResult.avg_inference_time_ms.toFixed(1) + " ms" : "—"
                color: qgcPal.color("qgc-blue")
            }
            AIStatCard { 
                label: qsTr("Max Inference Time"); 
                value: root.aiTimelineResult.max_inference_time_ms ? root.aiTimelineResult.max_inference_time_ms.toFixed(1) + " ms" : "—"
                color: qgcPal.color("qgc-red")
            }
            AIStatCard { 
                label: qsTr("Detections"); 
                value: root.aiTimelineResult.total_detections || "—"
                color: qgcPal.color("qgc-green")
            }
            AIStatCard { 
                label: qsTr("Replans"); 
                value: root.aiTimelineResult.replan_count || "—"
                color: qgcPal.color("qgc-orange")
            }
            AIStatCard { 
                label: qsTr("Faults"); 
                value: root.aiTimelineResult.fault_count || "—"
                color: root.aiTimelineResult.fault_count > 0 ? qgcPal.color("qgc-red") : qgcPal.color("qgc-green")
            }
        }
        
        // Inference time timeline
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Inference Time Timeline")
            
            ChartView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                antialiasing: true
                theme: ChartView.ChartThemeDark
                animationOptions: ChartView.SeriesAnimations
                
                LineSeries {
                    id: inferenceTimeSeries
                    name: qsTr("Inference Time (ms)")
                    color: qgcPal.color("qgc-blue")
                    useOpenGL: true
                }
                
                LineSeries {
                    id: thresholdLine
                    name: qsTr("Real-time Threshold (50ms)")
                    color: qgcPal.color("qgc-red")
                    useOpenGL: true
                }
                
                ValueAxis {
                    id: axisX
                    min: 0
                    max: 3600
                    titleText: qsTr("Time (s)")
                }
                ValueAxis {
                    id: axisY
                    min: 0
                    max: 100
                    titleText: qsTr("Inference Time (ms)")
                }
            }
        }
        
        // Detections by class
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Detections by Class")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.detectionsModel()
                
                TableViewColumn {
                    role: "class_name"
                    title: qsTr("Class")
                    width: ScreenTools.defaultFontPixelWidth * 20
                }
                TableViewColumn {
                    role: "count"
                    title: qsTr("Count")
                    width: ScreenTools.defaultFontPixelWidth * 12
                }
                TableViewColumn {
                    role: "avg_confidence"
                    title: qsTr("Avg Confidence")
                    width: ScreenTools.defaultFontPixelWidth * 15
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(1) + "%"
                        color: display > 0.8 ? qgcPal.color("qgc-green") : display > 0.5 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "avg_inference_time"
                    title: qsTr("Avg Time (ms)")
                    width: ScreenTools.defaultFontPixelWidth * 15
                    delegate: ItemDelegate { text: display.toFixed(1) }
                }
                TableViewColumn {
                    role: "first_seen"
                    title: qsTr("First Seen")
                    width: ScreenTools.defaultFontPixelWidth * 20
                    delegate: ItemDelegate { text: Qt.formatDateTime(new Date(display / 1000), "HH:mm:ss") }
                }
                TableViewColumn {
                    role: "last_seen"
                    title: qsTr("Last Seen")
                    width: ScreenTools.defaultFontPixelWidth * 20
                    delegate: ItemDelegate { text: Qt.formatDateTime(new Date(display / 1000), "HH:mm:ss") }
                }
            }
        }
        
        // Event timeline
        GroupBox {
            Layout.fillWidth: true
            Layout.fillHeight: true
            title: qsTr("AI Event Timeline")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.aiTimelineResult.events || []
                sortIndicatorVisible: true
                
                TableViewColumn {
                    role: "timestamp_us"
                    title: qsTr("Time")
                    width: ScreenTools.defaultFontPixelWidth * 20
                    delegate: ItemDelegate { 
                        text: Qt.formatDateTime(new Date(display / 1000), "HH:mm:ss.zzz")
                    }
                }
                TableViewColumn {
                    role: "vehicle_id"
                    title: qsTr("Vehicle")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { text: display ? "#" + display : "—" }
                }
                TableViewColumn {
                    role: "event_type"
                    title: qsTr("Type")
                    width: ScreenTools.defaultFontPixelWidth * 15
                }
                TableViewColumn {
                    role: "class_name"
                    title: qsTr("Class")
                    width: ScreenTools.defaultFontPixelWidth * 15
                }
                TableViewColumn {
                    role: "confidence"
                    title: qsTr("Confidence")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(1) + "%"
                        color: display > 0.8 ? qgcPal.color("qgc-green") : display > 0.5 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "inference_time_ms"
                    title: qsTr("Time (ms)")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { text: display.toFixed(1) }
                }
                TableViewColumn {
                    role: "details"
                    title: qsTr("Details")
                    width: ScreenTools.defaultFontPixelWidth * 30
                }
            }
        }
    }
    
    // Update inference time chart
    function updateInferenceChart() {
        inferenceTimeSeries.clear()
        thresholdLine.clear()
        
        if (aiTimelineResult.inference_timeline) {
            var t0 = aiTimelineResult.inference_timeline[0]?.timestamp_us || 0
            for (var i = 0; i < aiTimelineResult.inference_timeline.length; i++) {
                var e = aiTimelineResult.inference_timeline[i]
                var t = (e.timestamp_us - t0) / 1e6
                inferenceTimeSeries.append(t, e.inference_time_ms)
            }
        }
        
        // Real-time threshold line at 50ms
        thresholdLine.append(0, 50)
        thresholdLine.append(3600, 50)
    }
    
    function detectionsModel() {
        var model = []
        if (aiTimelineResult.detections_by_class) {
            for (var cls in aiTimelineResult.detections_by_class) {
                var d = aiTimelineResult.detections_by_class[cls]
                model.push({
                    class_name: cls,
                    count: d.count || 0,
                    avg_confidence: d.avg_confidence || 0,
                    avg_inference_time: d.avg_inference_time_ms || 0,
                    first_seen: d.first_seen_us || 0,
                    last_seen: d.last_seen_us || 0
                })
            }
        }
        return model
    }
    
    onAiTimelineResultChanged: {
        updateInferenceChart()
    }
    
    // Reusable stat card
    Component {
        id: AIStatCard
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: ScreenTools.defaultFontPixelWidth * 2
            color: qgcPal.window
            border.color: parent.color
            border.width: 2
            
            property string label: ""
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
                    text: label
                    font.pixelSize: ScreenTools.defaultFontPixelSize * 0.8
                    color: qgcPal.color("qgc-text-secondary")
                    horizontalAlignment: Text.AlignHCenter
                }
            }
        }
    }
}