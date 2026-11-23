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
    property var formationData: {}
    property var allVehicles: []
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Formation overview
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 10
            title: qsTr("Formation Overview")
            
            GridLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                columns: 4
                rows: 2
                flow: GridLayout.LeftToRight
                
                FormationMetric { label: qsTr("Type"); value: formationData.formation_type || "N/A" }
                FormationMetric { label: qsTr("State"); value: formationData.formation_state || "N/A" }
                FormationMetric { label: qsTr("Leader"); value: formationData.leader_id !== undefined ? "#" + formationData.leader_id : "N/A" }
                FormationMetric { label: qsTr("Members"); value: (formationData.member_ids ? formationData.member_ids.length : 0) }
                FormationMetric { label: qsTr("Avg Error"); value: (formationData.avg_formation_error_m || 0).toFixed(2) + "m" }
                FormationMetric { label: qsTr("Max Error"); value: (formationData.max_formation_error_m || 0).toFixed(2) + "m" }
                FormationMetric { label: qsTr("Locked"); value: formationData.formation_locked ? qsTr("Yes") : qsTr("No") }
                FormationMetric { label: qsTr("Updated"); value: formationData.last_update ? Qt.formatDateTime(new Date(formationData.last_update), "HH:mm:ss") : "N/A" }
            }
        }
        
        // Member positions table
        GroupBox {
            Layout.fillWidth: true
            Layout.fillHeight: true
            title: qsTr("Member Positions")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.allVehicles.filter(function(v) { 
                    return root.formationData.member_ids && root.formationData.member_ids.indexOf(v.id) >= 0 
                })
                
                TableViewColumn {
                    role: "id"
                    title: qsTr("ID")
                    width: ScreenTools.defaultFontPixelWidth * 8
                }
                TableViewColumn {
                    role: "callsign"
                    title: qsTr("Callsign")
                    width: ScreenTools.defaultFontPixelWidth * 12
                }
                TableViewColumn {
                    role: "is_leader"
                    title: qsTr("Leader")
                    width: ScreenTools.defaultFontPixelWidth * 8
                    delegate: ItemDelegate { text: display ? qsTr("Yes") : qsTr("No") }
                }
                TableViewColumn {
                    role: "lat"
                    title: qsTr("Lat")
                    width: ScreenTools.defaultFontPixelWidth * 14
                    delegate: ItemDelegate { text: display.toFixed(6) }
                }
                TableViewColumn {
                    role: "lon"
                    title: qsTr("Lon")
                    width: ScreenTools.defaultFontPixelWidth * 14
                    delegate: ItemDelegate { text: display.toFixed(6) }
                }
                TableViewColumn {
                    role: "alt_rel"
                    title: qsTr("Alt (m)")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { text: display.toFixed(1) }
                }
                TableViewColumn {
                    role: "formation_error_m"
                    title: qsTr("Error (m)")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display ? display.toFixed(2) : "N/A"
                        color: display > 5 ? qgcPal.color("qgc-red") : display > 2 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-green")
                    }
                }
                TableViewColumn {
                    role: "in_formation"
                    title: qsTr("In Form")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { text: display ? qsTr("Yes") : qsTr("No") }
                }
            }
        }
        
        // Formation error history chart
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Formation Error History")
            
            ChartView {
                anchors.fill: parent
                antialiasing: true
                theme: ChartView.ChartThemeDark
                animationOptions: ChartView.SeriesAnimations
                
                LineSeries {
                    id: avgErrorSeries
                    name: qsTr("Avg Error (m)")
                    color: qgcPal.color("qgc-blue")
                    useOpenGL: true
                }
                
                LineSeries {
                    id: maxErrorSeries
                    name: qsTr("Max Error (m)")
                    color: qgcPal.color("qgc-red")
                    useOpenGL: true
                }
                
                ValueAxis {
                    id: axisX
                    min: 0
                    max: 60  // 60 seconds
                    labelFormat: "%ds"
                    titleText: qsTr("Time (s)")
                }
                ValueAxis {
                    id: axisY
                    min: 0
                    max: 10
                    titleText: qsTr("Error (m)")
                }
            }
        }
    }
    
    // Update chart data periodically
    property var errorHistory: []
    property int historyIndex: 0
    
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            if (formationData && formationData.avg_formation_error_m !== undefined) {
                errorHistory.push({
                    time: historyIndex++,
                    avg: formationData.avg_formation_error_m,
                    max: formationData.max_formation_error_m
                })
                if (errorHistory.length > 60) errorHistory.shift()
                
                avgErrorSeries.clear()
                maxErrorSeries.clear()
                for (var i = 0; i < errorHistory.length; i++) {
                    avgErrorSeries.append(errorHistory[i].time, errorHistory[i].avg)
                    maxErrorSeries.append(errorHistory[i].time, errorHistory[i].max)
                }
                axisX.min = Math.max(0, historyIndex - 60)
                axisX.max = historyIndex
            }
        }
    }
    
    Component.onCompleted: {
        errorHistory = []
        historyIndex = 0
    }
}

// Reusable metric component
Component {
    id: FormationMetric
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
            color: qgcPal.color("qgc-text-primary")
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