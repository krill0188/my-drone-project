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
    property var coverageResult: {}
    
    signal analysisRequested(string logId, var params)
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Control panel
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 10
            title: qsTr("Coverage Analysis")
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                spacing: ScreenTools.defaultFontPixelWidth * 2
                
                RowLayout {
                    ToolButton {
                        text: qsTr("Analyze Coverage")
                        Layout.fillWidth: true
                        enabled: root.logId !== ""
                        onClicked: {
                            root.analysisRequested(root.logId, {
                                "grid_resolution_m": gridResolution.value,
                                "sensor_fov_deg": sensorFOV.value,
                                "flight_alt_m": flightAlt.value
                            })
                        }
                    }
                    
                    ToolButton {
                        text: qsTr("Export Heatmap")
                        Layout.fillWidth: true
                        enabled: Object.keys(root.coverageResult).length > 0
                        onClicked: {
                            // Export heatmap as image/CSV
                        }
                    }
                }
                
                RowLayout {
                    Label { text: qsTr("Grid Resolution (m):") }
                    SpinBox { id: gridResolution; minimum: 1; maximum: 100; value: 10; Layout.fillWidth: true }
                }
                RowLayout {
                    Label { text: qsTr("Sensor FOV (deg):") }
                    SpinBox { id: sensorFOV; minimum: 30; maximum: 180; value: 90; Layout.fillWidth: true }
                }
                RowLayout {
                    Label { text: qsTr("Flight Altitude (m):") }
                    SpinBox { id: flightAlt; minimum: 10; maximum: 500; value: 50; Layout.fillWidth: true }
                }
            }
        }
        
        // Summary cards
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 5
            spacing: ScreenTools.defaultFontPixelWidth * 2
            
            CoverageStatCard { 
                label: qsTr("Total Area"); 
                value: root.coverageResult.total_area_km2 ? root.coverageResult.total_area_km2.toFixed(2) + " km²" : "—" 
            }
            CoverageStatCard { 
                label: qsTr("Covered Area"); 
                value: root.coverageResult.covered_area_km2 ? root.coverageResult.covered_area_km2.toFixed(2) + " km²" : "—"
                color: qgcPal.color("qgc-blue")
            }
            CoverageStatCard { 
                label: qsTr("Coverage %"); 
                value: root.coverageResult.coverage_pct ? root.coverageResult.coverage_pct.toFixed(1) + "%" : "—"
                color: (root.coverageResult.coverage_pct || 0) > 80 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
            }
            CoverageStatCard { 
                label: qsTr("Grid Cells"); 
                value: root.coverageResult.grid_cells ? root.coverageResult.grid_cells : "—"
            }
            CoverageStatCard { 
                label: qsTr("Resolution"); 
                value: root.coverageResult.grid_resolution_m ? root.coverageResult.grid_resolution_m + " m" : "—"
            }
            CoverageStatCard { 
                label: qsTr("Overlap"); 
                value: root.coverageResult.overlap_pct ? root.coverageResult.overlap_pct.toFixed(1) + "%" : "—"
            }
        }
        
        // Heatmap visualization
        GroupBox {
            Layout.fillWidth: true
            Layout.fillHeight: true
            title: qsTr("Coverage Heatmap")
            
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: qgcPal.window
                border.color: qgcPal.color("qgc-border")
                border.width: 1
                
                // Would render heatmap here using Canvas or custom QQuickItem
                // For now, show placeholder
                Label {
                    anchors.centerIn: parent
                    text: root.coverageResult.heatmap_data ? 
                          qsTr("Heatmap rendered (%1 x %2 grid)").arg(root.coverageResult.grid_width || 0).arg(root.coverageResult.grid_height || 0) :
                          qsTr("Run analysis to generate heatmap")
                    color: qgcPal.color("qgc-text-secondary")
                }
            }
        }
        
        // Sensor coverage breakdown
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 10
            title: qsTr("Per-Sensor Coverage")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.sensorCoverageModel()
                
                TableViewColumn {
                    role: "sensor"
                    title: qsTr("Sensor")
                    width: ScreenTools.defaultFontPixelWidth * 20
                }
                TableViewColumn {
                    role: "coverage_pct"
                    title: qsTr("Coverage %")
                    width: ScreenTools.defaultFontPixelWidth * 15
                    delegate: ItemDelegate { 
                        text: display.toFixed(1) + "%"
                        color: display > 80 ? qgcPal.color("qgc-green") : display > 50 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "area_km2"
                    title: qsTr("Area (km²)")
                    width: ScreenTools.defaultFontPixelWidth * 15
                    delegate: ItemDelegate { text: display.toFixed(3) }
                }
                TableViewColumn {
                    role: "gaps"
                    title: qsTr("Gaps")
                    width: ScreenTools.defaultFontPixelWidth * 10
                }
            }
        }
    }
    
    // Helper function for sensor coverage model
    function sensorCoverageModel() {
        var model = []
        if (coverageResult.sensor_coverage) {
            for (var sensor in coverageResult.sensor_coverage) {
                model.push({
                    sensor: sensor,
                    coverage_pct: coverageResult.sensor_coverage[sensor],
                    area_km2: coverageResult.sensor_area_km2 ? coverageResult.sensor_area_km2[sensor] : 0,
                    gaps: coverageResult.sensor_gaps ? coverageResult.sensor_gaps[sensor] : 0
                })
            }
        }
        return model
    }
    
    // Reusable stat card
    Component {
        id: CoverageStatCard
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