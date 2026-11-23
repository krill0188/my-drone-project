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
    property var qosResult: {}
    
    signal analysisRequested(string logId)
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Control panel
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 8
            title: qsTr("QoS Analysis")
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                spacing: ScreenTools.defaultFontPixelWidth * 2
                
                ToolButton {
                    text: qsTr("Analyze QoS")
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
            
            QoSStatCard { 
                label: qsTr("Avg Link Quality"); 
                value: root.qosResult.avg_link_quality ? (root.qosResult.avg_link_quality * 100).toFixed(1) + "%" : "—" 
            }
            QoSStatCard { 
                label: qsTr("Min Link Quality"); 
                value: root.qosResult.min_link_quality ? (root.qosResult.min_link_quality * 100).toFixed(1) + "%" : "—"
                color: qgcPal.color("qgc-red")
            }
            QoSStatCard { 
                label: qsTr("Avg Latency"); 
                value: root.qosResult.avg_latency_ms ? root.qosResult.avg_latency_ms.toFixed(1) + " ms" : "—"
                color: qgcPal.color("qgc-blue")
            }
            QoSStatCard { 
                label: qsTr("Packet Loss Rate"); 
                value: root.qosResult.loss_rate ? (root.qosResult.loss_rate * 100).toFixed(2) + "%" : "—"
                color: (root.qosResult.loss_rate || 0) > 0.05 ? qgcPal.color("qgc-red") : qgcPal.color("qgc-green")
            }
            QoSStatCard { 
                label: qsTr("Total Packets"); 
                value: root.qosResult.total_packets || "—"
            }
            QoSStatCard { 
                label: qsTr("Lost Packets"); 
                value: root.qosResult.lost_packets || "—"
                color: qgcPal.color("qgc-red")
            }
            QoSStatCard { 
                label: qsTr("Throughput"); 
                value: root.qosResult.avg_throughput_mbps ? root.qosResult.avg_throughput_mbps.toFixed(1) + " Mbps" : "—"
                color: qgcPal.color("qgc-green")
            }
            QoSStatCard { 
                label: qsTr("Critical Links"); 
                value: root.qosResult.critical_links_count || "—"
                color: (root.qosResult.critical_links_count || 0) > 0 ? qgcPal.color("qgc-red") : qgcPal.color("qgc-green")
            }
        }
        
        // Link quality timeline
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Link Quality Timeline")
            
            ChartView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                antialiasing: true
                theme: ChartView.ChartThemeDark
                animationOptions: ChartView.SeriesAnimations
                
                LineSeries {
                    id: linkQualitySeries
                    name: qsTr("Avg Link Quality")
                    color: qgcPal.color("qgc-blue")
                    useOpenGL: true
                }
                
                LineSeries {
                    id: minLinkQualitySeries
                    name: qsTr("Min Link Quality")
                    color: qgcPal.color("qgc-red")
                    useOpenGL: true
                }
                
                LineSeries {
                    id: latencySeries
                    name: qsTr("Latency (ms)")
                    color: qgcPal.color("qgc-yellow")
                    useOpenGL: true
                }
                
                LineSeries {
                    id: lossSeries
                    name: qsTr("Packet Loss %")
                    color: qgcPal.color("qgc-orange")
                    useOpenGL: true
                }
                
                ValueAxis {
                    id: axisX
                    min: 0
                    max: 3600
                    titleText: qsTr("Time (s)")
                }
                ValueAxis {
                    id: axisY1
                    min: 0
                    max: 1
                    titleText: qsTr("Quality / Loss %")
                    labelFormat: "%.0%"
                }
                ValueAxis {
                    id: axisY2
                    min: 0
                    max: 200
                    titleText: qsTr("Latency (ms)")
                }
            }
        }
        
        // Per-link quality matrix
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Per-Link Quality Matrix")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.perLinkModel()
                
                TableViewColumn {
                    role: "from"
                    title: qsTr("From")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "to"
                    title: qsTr("To")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "avg_quality"
                    title: qsTr("Avg Quality")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(1) + "%"
                        color: display > 0.7 ? qgcPal.color("qgc-green") : display > 0.4 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "min_quality"
                    title: qsTr("Min Quality")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(1) + "%"
                        color: display > 0.5 ? qgcPal.color("qgc-green") : display > 0.3 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "avg_rssi"
                    title: qsTr("Avg RSSI")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: display.toFixed(0) + " dBm"
                        color: display > -70 ? qgcPal.color("qgc-green") : display > -85 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "avg_latency"
                    title: qsTr("Latency (ms)")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: display.toFixed(1)
                        color: display < 50 ? qgcPal.color("qgc-green") : display < 100 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "loss_rate"
                    title: qsTr("Loss %")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(2) + "%"
                        color: display > 0.05 ? qgcPal.color("qgc-red") : display > 0.01 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-green")
                    }
                }
                TableViewColumn {
                    role: "throughput"
                    title: qsTr("Throughput")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { text: display.toFixed(2) + " Mbps" }
                }
            }
        }
        
        // Critical links
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 10
            title: qsTr("Critical Links (Quality < 30% or Loss > 10%)")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.criticalLinksModel()
                
                TableViewColumn {
                    role: "from"
                    title: qsTr("From")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "to"
                    title: qsTr("To")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "quality"
                    title: qsTr("Quality")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(1) + "%"
                        color: qgcPal.color("qgc-red")
                        font.bold: true
                    }
                }
                TableViewColumn {
                    role: "rssi"
                    title: qsTr("RSSI")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { text: display.toFixed(0) + " dBm" }
                }
                TableViewColumn {
                    role: "latency"
                    title: qsTr("Latency")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { text: display.toFixed(1) + " ms" }
                }
                TableViewColumn {
                    role: "loss_rate"
                    title: qsTr("Loss %")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(2) + "%"
                        color: qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "recommendation"
                    title: qsTr("Recommendation")
                    width: ScreenTools.defaultFontPixelWidth * 30
                }
            }
        }
        
        // Recommendations
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 10
            title: qsTr("QoS Recommendations")
            
            ListView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                model: root.qosResult.recommendations || []
                delegate: ItemDelegate {
                    width: ListView.view.width
                    text: "• " + modelData
                    font.pixelSize: ScreenTools.defaultFontPixelSize
                    padding: ScreenTools.defaultFontPixelWidth * 2
                }
            }
        }
    }
    
    function perLinkModel() {
        var model = []
        if (qosResult.per_link_quality) {
            for (var key in qosResult.per_link_quality) {
                var parts = key.split("->")
                if (parts.length === 2) {
                    var from = parts[0]
                    var to = parts[1]
                    var q = qosResult.per_link_quality[key]
                    model.push({
                        from: from,
                        to: to,
                        avg_quality: q.avg_quality || 0,
                        min_quality: q.min_quality || 0,
                        avg_rssi: q.avg_rssi || -100,
                        avg_latency: q.avg_latency_ms || 0,
                        loss_rate: q.loss_rate || 0,
                        throughput: q.avg_throughput_mbps || 0
                    })
                }
            }
        }
        return model
    }
    
    function criticalLinksModel() {
        var model = []
        if (qosResult.critical_links) {
            for (var i = 0; i < qosResult.critical_links.length; i++) {
                var l = qosResult.critical_links[i]
                model.push({
                    from: l.from || 0,
                    to: l.to || 0,
                    quality: l.quality || 0,
                    rssi: l.rssi || -100,
                    latency: l.latency || 0,
                    loss_rate: l.loss_rate || 0,
                    recommendation: l.recommendation || "Investigate link"
                })
            }
        }
        return model
    }
    
    // Reusable stat card
    Component {
        id: QoSStatCard
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