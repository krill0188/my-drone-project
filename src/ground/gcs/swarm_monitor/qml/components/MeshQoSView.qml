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
    property SwarmMonitor monitor: null
    property var meshQoSData: []
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Mesh overview
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 10
            title: qsTr("Mesh Network Overview")
            
            GridLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                columns: 4
                rows: 2
                flow: GridLayout.LeftToRight
                
                QoSMetric { label: qsTr("Nodes"); value: root.totalNodes() }
                QoSMetric { label: qsTr("Active Links"); value: root.totalActiveLinks() }
                QoSMetric { label: qsTr("Avg Quality"); value: (root.averageQuality() * 100).toFixed(1) + "%" }
                QoSMetric { label: qsTr("Min Quality"); value: (root.minimumQuality() * 100).toFixed(1) + "%" }
                QoSMetric { label: qsTr("Avg Latency"); value: root.averageLatency().toFixed(1) + "ms" }
                QoSMetric { label: qsTr("Max Latency"); value: root.maximumLatency().toFixed(1) + "ms" }
                QoSMetric { label: qsTr("Total Rx"); value: root.totalPacketsReceived() }
                QoSMetric { label: qsTr("Total Lost"); value: root.totalPacketsLost() }
            }
        }
        
        // Link quality matrix
        GroupBox {
            Layout.fillWidth: true
            Layout.fillHeight: true
            title: qsTr("Link Quality Matrix")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.meshQoSData
                
                TableViewColumn {
                    role: "node_id"
                    title: qsTr("Node")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "radio_type"
                    title: qsTr("Radio")
                    width: ScreenTools.defaultFontPixelWidth * 12
                }
                TableViewColumn {
                    role: "channel"
                    title: qsTr("Ch")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "bandwidth_mhz"
                    title: qsTr("BW (MHz)")
                    width: ScreenTools.defaultFontPixelWidth * 10
                }
                TableViewColumn {
                    role: "tx_power_dbm"
                    title: qsTr("TX (dBm)")
                    width: ScreenTools.defaultFontPixelWidth * 10
                }
                TableViewColumn {
                    role: "avg_link_quality"
                    title: qsTr("Avg Quality")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(1) + "%"
                        color: display > 0.7 ? qgcPal.color("qgc-green") : display > 0.4 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "min_link_quality"
                    title: qsTr("Min Quality")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(1) + "%"
                        color: display > 0.5 ? qgcPal.color("qgc-green") : display > 0.3 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "active_links"
                    title: qsTr("Links")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "total_neighbors"
                    title: qsTr("Neighbors")
                    width: ScreenTools.defaultFontPixelWidth * 10
                }
            }
        }
        
        // Link quality heatmap (simplified as bar chart)
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 20
            title: qsTr("Per-Link Quality")
            
            ChartView {
                anchors.fill: parent
                antialiasing: true
                theme: ChartView.ChartThemeDark
                animationOptions: ChartView.SeriesAnimations
                
                BarSeries {
                    id: linkQualitySeries
                }
                
                BarSeries {
                    id: rssiSeries
                }
                
                ValueAxis {
                    id: axisY
                    min: 0
                    max: 100
                    titleText: qsTr("Quality (%) / RSSI (dBm)")
                }
                BarCategoryAxis {
                    id: axisX
                    categories: []
                }
            }
        }
        
        // Link details
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Link Details")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.expandedLinks()
                
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
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(1) + "%"
                        color: display > 0.7 ? qgcPal.color("qgc-green") : display > 0.4 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "rssi"
                    title: qsTr("RSSI (dBm)")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: display.toFixed(0)
                        color: display > -70 ? qgcPal.color("qgc-green") : display > -85 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "latency"
                    title: qsTr("Latency (ms)")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: display.toFixed(1)
                        color: display < 50 ? qgcPal.color("qgc-green") : display < 100 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "packets_sent"
                    title: qsTr("Sent")
                    width: ScreenTools.defaultFontPixelWidth * 10
                }
                TableViewColumn {
                    role: "packets_received"
                    title: qsTr("Received")
                    width: ScreenTools.defaultFontPixelWidth * 10
                }
                TableViewColumn {
                    role: "packets_lost"
                    title: qsTr("Lost")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display
                        color: display > 0 ? qgcPal.color("qgc-red") : qgcPal.color("qgc-green")
                    }
                }
                TableViewColumn {
                    role: "loss_rate"
                    title: qsTr("Loss %")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(2) + "%"
                        color: display > 0.05 ? qgcPal.color("qgc-red") : display > 0.01 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-green")
                    }
                }
            }
        }
    }
    
    // Helper functions
    function totalNodes() {
        return meshQoSData.length
    }
    
    function totalActiveLinks() {
        var sum = 0
        for (var i = 0; i < meshQoSData.length; i++) {
            sum += meshQoSData[i].active_links || 0
        }
        return sum
    }
    
    function averageQuality() {
        if (meshQoSData.length === 0) return 1
        var sum = 0
        for (var i = 0; i < meshQoSData.length; i++) {
            sum += meshQoSData[i].avg_link_quality || 1
        }
        return sum / meshQoSData.length
    }
    
    function minimumQuality() {
        if (meshQoSData.length === 0) return 1
        var min = 1
        for (var i = 0; i < meshQoSData.length; i++) {
            var q = meshQoSData[i].min_link_quality || 1
            if (q < min) min = q
        }
        return min
    }
    
    function averageLatency() {
        if (meshQoSData.length === 0) return 0
        var sum = 0
        var count = 0
        for (var i = 0; i < meshQoSData.length; i++) {
            var m = meshQoSData[i]
            for (var key in m.latency_ms_to) {
                sum += m.latency_ms_to[key]
                count++
            }
        }
        return count > 0 ? sum / count : 0
    }
    
    function maximumLatency() {
        var max = 0
        for (var i = 0; i < meshQoSData.length; i++) {
            var m = meshQoSData[i]
            for (var key in m.latency_ms_to) {
                if (m.latency_ms_to[key] > max) max = m.latency_ms_to[key]
            }
        }
        return max
    }
    
    function totalPacketsReceived() {
        var sum = 0
        for (var i = 0; i < meshQoSData.length; i++) {
            var m = meshQoSData[i]
            for (var key in m.packets_received_from) {
                sum += m.packets_received_from[key]
            }
        }
        return sum
    }
    
    function totalPacketsLost() {
        var sum = 0
        for (var i = 0; i < meshQoSData.length; i++) {
            var m = meshQoSData[i]
            for (var key in m.packets_lost_to) {
                sum += m.packets_lost_to[key]
            }
        }
        return sum
    }
    
    function expandedLinks() {
        var links = []
        for (var i = 0; i < meshQoSData.length; i++) {
            var m = meshQoSData[i]
            for (var key in m.link_quality_to) {
                var sent = m.packets_sent_to[key] || 0
                var recv = m.packets_received_from[key] || 0
                var lost = m.packets_lost_to[key] || 0
                links.push({
                    from: m.node_id,
                    to: parseInt(key),
                    quality: m.link_quality_to[key] || 0,
                    rssi: m.rssi_dbm_to[key] || -100,
                    latency: m.latency_ms_to[key] || 0,
                    packets_sent: sent,
                    packets_received: recv,
                    packets_lost: lost,
                    loss_rate: sent > 0 ? lost / sent : 0
                })
            }
        }
        return links
    }
    
    // Reusable metric component
    Component {
        id: QoSMetric
        Column {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: ScreenTools.defaultFontPixelWidth
            horizontalAlignment: Text.AlignHCenter
            
            property string label: ""
            property string value: ""
            
            Text {
                text: value
                font.pixelSize: ScreenTools.defaultFontPixelSize * 1.5
                font.bold: true
                color: qgcPal.color("qgc-primary")
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