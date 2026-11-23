import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
import QtCharts 2.15
import QtPositioning 5.15
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.FactSystem 1.0
import QGroundControl.FlightMap 1.0
import QGroundControl.ScreenTools 1.0
import QGroundControl.Palette 1.0
import SwarmMonitor 1.0

Item {
    id: root
    property SwarmMonitor monitor: null
    property int refreshRate: 100  // ms
    property var selectedVehicleId: -1
    property var safetyEventsModel: []
    property var formationErrorHistory: []
    property var linkQualityHistory: []
    
    // Timer for periodic updates
    Timer {
        interval: root.refreshRate
        running: true
        repeat: true
        onTriggered: {
            if (monitor) {
                summaryData = monitor.getSwarmSummary()
                allVehicles = monitor.getAllVehicleStatus()
                formationData = monitor.getFormationStatus()
                meshQoSData = monitor.getAllMeshQoS()
                recentEvents = monitor.getSafetyEvents(50)
            }
        }
    }
    
    property var summaryData: {}
    property var allVehicles: []
    property var formationData: {}
    property var meshQoSData: []
    property var recentEvents: []
    
    ColumnLayout {
        anchors.fill: parent
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Toolbar
        ToolBar {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 3
            
            ToolButton {
                text: qsTr("ESTOP ALL")
                iconSource: "qrc:/qmlIcons/stopCircle.svg"
                background: Rectangle { color: qgcPal.color("qgc-red"); radius: 4 }
                font.bold: true
                onClicked: {
                    if (monitor && confirm(qsTr("EMERGENCY STOP ALL DRONES?"))) {
                        monitor.sendEstop(0)
                    }
                }
            }
            
            ToolButton {
                text: qsTr("RTL ALL")
                iconSource: "qrc:/qmlIcons/home.svg"
                background: Rectangle { color: qgcPal.color("qgc-orange"); radius: 4 }
                onClicked: {
                    if (monitor && confirm(qsTr("Return all drones to launch?"))) {
                        monitor.sendRTL(0)
                    }
                }
            }
            
            ToolButton {
                text: qsTr("HOLD ALL")
                iconSource: "qrc:/qmlIcons/pauseCircle.svg"
                background: Rectangle { color: qgcPal.color("qgc-yellow"); radius: 4 }
                onClicked: {
                    if (monitor) monitor.sendHold(0)
                }
            }
            
            Item { Layout.fillWidth: true }
            
            ComboBox {
                id: formationSelector
                model: ["line", "vee", "diamond", "grid", "custom"]
                currentText: formationData.formation_type || "line"
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 15
                onActivated: {
                    if (monitor) monitor.setFormationType(model[index])
                }
            }
            
            ToolButton {
                text: qsTr("Apply Formation")
                enabled: formationSelector.currentText !== (formationData.formation_type || "")
                onClicked: {
                    if (monitor) monitor.setFormationType(formationSelector.currentText)
                }
            }
            
            Label {
                id: pythonStatusLabel
                text: monitor && monitor.isPythonBackendAvailable() ? 
                      qsTr("Python: Connected") : qsTr("Python: Disconnected")
                color: monitor && monitor.isPythonBackendAvailable() ? qgcPal.color("qgc-green") : qgcPal.color("qgc-red")
                font.bold: true
            }
        }
        
        // Summary cards
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 5
            spacing: ScreenTools.defaultFontPixelWidth * 2
            
            SummaryCard {
                Layout.fillWidth: true
                title: qsTr("Total Vehicles")
                value: summaryData.total_vehicles || 0
                icon: "qrc:/qmlIcons/drone.svg"
                color: qgcPal.color("qgc-primary")
            }
            SummaryCard {
                Layout.fillWidth: true
                title: qsTr("Armed")
                value: summaryData.armed_count || 0
                icon: "qrc:/qmlIcons/shieldCheck.svg"
                color: summaryData.armed_count > 0 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-gray")
            }
            SummaryCard {
                Layout.fillWidth: true
                title: qsTr("Flying")
                value: summaryData.flying_count || 0
                icon: "qrc:/qmlIcons/arrowUpRight.svg"
                color: summaryData.flying_count > 0 ? qgcPal.color("qgc-blue") : qgcPal.color("qgc-gray")
            }
            SummaryCard {
                Layout.fillWidth: true
                title: qsTr("Avg Battery")
                value: (summaryData.avg_battery_pct || 0).toFixed(1) + "%"
                icon: "qrc:/qmlIcons/battery.svg"
                color: (summaryData.avg_battery_pct || 100) > 30 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-red")
            }
            SummaryCard {
                Layout.fillWidth: true
                title: qsTr("Avg Link")
                value: ((summaryData.avg_link_quality || 1) * 100).toFixed(0) + "%"
                icon: "qrc:/qmlIcons/signal.svg"
                color: (summaryData.avg_link_quality || 1) > 0.7 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
            }
            SummaryCard {
                Layout.fillWidth: true
                title: qsTr("Formation Err")
                value: (summaryData.avg_formation_error_m || 0).toFixed(1) + "m"
                icon: "qrc:/qmlIcons/ruler.svg"
                color: (summaryData.max_formation_error_m || 0) > 5 ? qgcPal.color("qgc-red") : qgcPal.color("qgc-green")
            }
            SummaryCard {
                Layout.fillWidth: true
                title: qsTr("Safety Events")
                value: summaryData.unacknowledged_safety_events || 0
                icon: "qrc:/qmlIcons/alertTriangle.svg"
                color: (summaryData.critical_safety_events || 0) > 0 ? qgcPal.color("qgc-red") : qgcPal.color("qgc-green")
            }
        }
        
        // Main content tabs
        TabBar {
            id: tabBar
            Layout.fillWidth: true
            TabButton { text: qsTr("Formation") }
            TabButton { text: qsTr("Health") }
            TabButton { text: qsTr("Mesh QoS") }
            TabButton { text: qsTr("Safety") }
            TabButton { text: qsTr("Map") }
        }
        
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabBar.currentIndex
            
            // Formation Tab
            FormationView {
                monitor: root.monitor
                formationData: root.formationData
                allVehicles: root.allVehicles
            }
            
            // Health Tab
            HealthView {
                monitor: root.monitor
                allVehicles: root.allVehicles
            }
            
            // Mesh QoS Tab
            MeshQoSView {
                monitor: root.monitor
                meshQoSData: root.meshQoSData
            }
            
            // Safety Tab
            SafetyEventView {
                monitor: root.monitor
                safetyEventsModel: root.recentEvents
                onAcknowledgeEvent: {
                    if (monitor) monitor.acknowledgeSafetyEvent(timestamp, ackBy)
                }
            }
            
            // Map Tab
            FlightMap {
                // Swarm vehicles would be shown here
            }
        }
    }
    
    // Connections
    Connections {
        target: monitor
        onVehicleStatusUpdated: {
            // Update handled by timer
        }
        onFormationStatusUpdated: {
            // Update handled by timer
        }
        onMeshQoSUpdated: {
            // Update handled by timer
        }
        onSafetyEventReceived: {
            console.log("New safety event:", event)
            // Could show toast notification
        }
        onSwarmSummaryUpdated: {
            summaryData = summary
        }
        onPythonBackendStatusChanged: {
            pythonStatusLabel.text = available ? qsTr("Python: Connected") : qsTr("Python: Disconnected")
        }
        onErrorOccurred: {
            console.error("Swarm Monitor Error:", message)
        }
    }
    
    Component.onCompleted: {
        if (monitor) {
            // Initial load
            summaryData = monitor.getSwarmSummary()
            allVehicles = monitor.getAllVehicleStatus()
            formationData = monitor.getFormationStatus()
            meshQoSData = monitor.getAllMeshQoS()
            recentEvents = monitor.getSafetyEvents(50)
        }
    }
}

// Reusable Summary Card Component
Component {
    id: SummaryCard
    Rectangle {
        Layout.fillWidth: true
        Layout.fillHeight: true
        radius: ScreenTools.defaultFontPixelWidth * 2
        color: qgcPal.window
        border.color: qgcPal.color("qgc-border")
        border.width: 1
        
        property string title: ""
        property string value: ""
        property string icon: ""
        property color color: qgcPal.color("qgc-primary")
        
        Column {
            anchors.centerIn: parent
            spacing: ScreenTools.defaultFontPixelWidth
            
            Image {
                source: icon
                width: ScreenTools.defaultFontPixelHeight * 2
                height: ScreenTools.defaultFontPixelHeight * 2
                color: parent.color
            }
            
            Text {
                text: value
                font.pixelSize: ScreenTools.defaultFontPixelSize * 2
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