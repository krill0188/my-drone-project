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
    property var allVehicles: []
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Fleet summary
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 12
            title: qsTr("Fleet Health Summary")
            
            GridLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                columns: 4
                rows: 3
                flow: GridLayout.LeftToRight
                
                HealthMetric { 
                    label: qsTr("Avg Battery"); 
                    value: root.averageBattery().toFixed(1) + "%"
                    color: root.averageBattery() > 30 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-red")
                }
                HealthMetric { 
                    label: qsTr("Min Battery"); 
                    value: root.minimumBattery().toFixed(1) + "%"
                    color: root.minimumBattery() > 20 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
                }
                HealthMetric { 
                    label: qsTr("Avg Link"); 
                    value: (root.averageLink() * 100).toFixed(0) + "%"
                    color: root.averageLink() > 0.7 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
                }
                HealthMetric { 
                    label: qsTr("Min Link"); 
                    value: (root.minimumLink() * 100).toFixed(0) + "%"
                    color: root.minimumLink() > 0.5 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-red")
                }
                HealthMetric { 
                    label: qsTr("GPS Sats (Avg)"); 
                    value: root.averageGpsSats().toFixed(0)
                    color: root.averageGpsSats() > 10 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
                }
                HealthMetric { 
                    label: qsTr("GPS Fix OK"); 
                    value: root.gpsHealthyCount() + "/" + root.allVehicles.length
                    color: root.gpsHealthyCount() === root.allVehicles.length ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
                }
                HealthMetric { 
                    label: qsTr("EKF Healthy"); 
                    value: root.ekfHealthyCount() + "/" + root.allVehicles.length
                    color: root.ekfHealthyCount() === root.allVehicles.length ? qgcPal.color("qgc-green") : qgcPal.color("qgc-red")
                }
                HealthMetric { 
                    label: qsTr("Flying"); 
                    value: root.flyingCount() + "/" + root.allVehicles.length
                    color: qgcPal.color("qgc-blue")
                }
                HealthMetric { 
                    label: qsTr("Armed"); 
                    value: root.armedCount() + "/" + root.allVehicles.length
                    color: qgcPal.color("qgc-primary")
                }
                HealthMetric { 
                    label: qsTr("Detections"); 
                    value: root.totalDetections()
                    color: qgcPal.color("qgc-purple")
                }
                HealthMetric { 
                    label: qsTr("Avg Inference"); 
                    value: root.averageInference().toFixed(1) + "ms"
                    color: root.averageInference() < 100 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
                }
                HealthMetric { 
                    label: qsTr("Critical Events"); 
                    value: root.criticalEventCount()
                    color: root.criticalEventCount() > 0 ? qgcPal.color("qgc-red") : qgcPal.color("qgc-green")
                }
            }
        }
        
        // Vehicle detail table
        GroupBox {
            Layout.fillWidth: true
            Layout.fillHeight: true
            title: qsTr("Vehicle Details")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.allVehicles
                sortIndicatorVisible: true
                
                TableViewColumn {
                    role: "id"
                    title: qsTr("ID")
                    width: ScreenTools.defaultFontPixelWidth * 6
                }
                TableViewColumn {
                    role: "callsign"
                    title: qsTr("Callsign")
                    width: ScreenTools.defaultFontPixelWidth * 12
                }
                TableViewColumn {
                    role: "flight_mode"
                    title: qsTr("Mode")
                    width: ScreenTools.defaultFontPixelWidth * 12
                }
                TableViewColumn {
                    role: "arming_state"
                    title: qsTr("Armed")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "battery_remaining_pct"
                    title: qsTr("Batt %")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display.toFixed(1) + "%"
                        color: display > 30 ? qgcPal.color("qgc-green") : display > 20 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "battery_voltage_v"
                    title: qsTr("Voltage")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { text: display.toFixed(2) + "V" }
                }
                TableViewColumn {
                    role: "battery_current_a"
                    title: qsTr("Current")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { text: display.toFixed(1) + "A" }
                }
                TableViewColumn {
                    role: "gps_satellites"
                    title: qsTr("Sats")
                    width: ScreenTools.defaultFontPixelWidth * 8
                    delegate: ItemDelegate { 
                        text: display
                        color: display >= 10 ? qgcPal.color("qgc-green") : display >= 6 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "gps_hdop"
                    title: qsTr("HDOP")
                    width: ScreenTools.defaultFontPixelWidth * 8
                    delegate: ItemDelegate { 
                        text: display.toFixed(1)
                        color: display < 1.5 ? qgcPal.color("qgc-green") : display < 2.5 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "link_quality"
                    title: qsTr("Link")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: (display * 100).toFixed(0) + "%"
                        color: display > 0.7 ? qgcPal.color("qgc-green") : display > 0.4 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "rssi_dbm"
                    title: qsTr("RSSI")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display.toFixed(0) + "dBm"
                        color: display > -70 ? qgcPal.color("qgc-green") : display > -85 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "latency_ms"
                    title: qsTr("Latency")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display.toFixed(0) + "ms"
                        color: display < 50 ? qgcPal.color("qgc-green") : display < 100 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "ekf_innovation"
                    title: qsTr("EKF Innov")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display.toFixed(3)
                        color: display < 0.5 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
                    }
                }
                TableViewColumn {
                    role: "active_detections"
                    title: qsTr("Detects")
                    width: ScreenTools.defaultFontPixelWidth * 10
                }
                TableViewColumn {
                    role: "inference_time_ms"
                    title: qsTr("Infer (ms)")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display.toFixed(1)
                        color: display < 50 ? qgcPal.color("qgc-green") : display < 100 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "formation_error_m"
                    title: qsTr("Form Err")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display ? display.toFixed(2) + "m" : "N/A"
                        color: display > 5 ? qgcPal.color("qgc-red") : display > 2 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-green")
                    }
                }
            }
        }
    }
    
    // Helper functions
    function averageBattery() {
        if (allVehicles.length === 0) return 0
        var sum = 0
        for (var i = 0; i < allVehicles.length; i++) {
            sum += allVehicles[i].battery_remaining_pct || 0
        }
        return sum / allVehicles.length
    }
    
    function minimumBattery() {
        if (allVehicles.length === 0) return 100
        var min = 100
        for (var i = 0; i < allVehicles.length; i++) {
            var b = allVehicles[i].battery_remaining_pct || 100
            if (b < min) min = b
        }
        return min
    }
    
    function averageLink() {
        if (allVehicles.length === 0) return 1
        var sum = 0
        for (var i = 0; i < allVehicles.length; i++) {
            sum += allVehicles[i].link_quality || 1
        }
        return sum / allVehicles.length
    }
    
    function minimumLink() {
        if (allVehicles.length === 0) return 1
        var min = 1
        for (var i = 0; i < allVehicles.length; i++) {
            var l = allVehicles[i].link_quality || 1
            if (l < min) min = l
        }
        return min
    }
    
    function averageGpsSats() {
        if (allVehicles.length === 0) return 0
        var sum = 0
        for (var i = 0; i < allVehicles.length; i++) {
            sum += allVehicles[i].gps_satellites || 0
        }
        return sum / allVehicles.length
    }
    
    function gpsHealthyCount() {
        var count = 0
        for (var i = 0; i < allVehicles.length; i++) {
            if (allVehicles[i].gps_healthy) count++
        }
        return count
    }
    
    function ekfHealthyCount() {
        var count = 0
        for (var i = 0; i < allVehicles.length; i++) {
            if (allVehicles[i].ekf_healthy) count++
        }
        return count
    }
    
    function flyingCount() {
        var count = 0
        for (var i = 0; i < allVehicles.length; i++) {
            if (allVehicles[i].is_flying) count++
        }
        return count
    }
    
    function armedCount() {
        var count = 0
        for (var i = 0; i < allVehicles.length; i++) {
            if (allVehicles[i].arming_state === "ARMED") count++
        }
        return count
    }
    
    function totalDetections() {
        var sum = 0
        for (var i = 0; i < allVehicles.length; i++) {
            sum += allVehicles[i].active_detections || 0
        }
        return sum
    }
    
    function averageInference() {
        if (allVehicles.length === 0) return 0
        var sum = 0
        for (var i = 0; i < allVehicles.length; i++) {
            sum += allVehicles[i].inference_time_ms || 0
        }
        return sum / allVehicles.length
    }
    
    function criticalEventCount() {
        // Would check safety events
        return 0
    }
    
    // Reusable metric component
    Component {
        id: HealthMetric
        Column {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: ScreenTools.defaultFontPixelWidth
            horizontalAlignment: Text.AlignHCenter
            
            property string label: ""
            property string value: ""
            property color color: qgcPal.color("qgc-primary")
            
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