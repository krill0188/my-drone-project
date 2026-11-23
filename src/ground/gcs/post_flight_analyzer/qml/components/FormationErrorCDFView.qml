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
    property var formationResult: {}
    
    signal analysisRequested(string logId)
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Control panel
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 8
            title: qsTr("Formation Error Analysis")
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                spacing: ScreenTools.defaultFontPixelWidth * 2
                
                ToolButton {
                    text: qsTr("Analyze Formation Error")
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
            
            CDFStatCard { 
                label: qsTr("Mean Error"); 
                value: root.formationResult.mean_error_m ? root.formationResult.mean_error_m.toFixed(2) + " m" : "—" 
            }
            CDFStatCard { 
                label: qsTr("Median Error"); 
                value: root.formationResult.median_error_m ? root.formationResult.median_error_m.toFixed(2) + " m" : "—"
                color: qgcPal.color("qgc-blue")
            }
            CDFStatCard { 
                label: qsTr("Std Dev"); 
                value: root.formationResult.std_error_m ? root.formationResult.std_error_m.toFixed(2) + " m" : "—"
                color: qgcPal.color("qgc-purple")
            }
            CDFStatCard { 
                label: qsTr("Max Error"); 
                value: root.formationResult.max_error_m ? root.formationResult.max_error_m.toFixed(2) + " m" : "—"
                color: qgcPal.color("qgc-red")
            }
            CDFStatCard { 
                label: qsTr("P95 Error"); 
                value: root.formationResult.p95_error_m ? root.formationResult.p95_error_m.toFixed(2) + " m" : "—"
                color: qgcPal.color("qgc-orange")
            }
            CDFStatCard { 
                label: qsTr("P99 Error"); 
                value: root.formationResult.p99_error_m ? root.formationResult.p99_error_m.toFixed(2) + " m" : "—"
                color: qgcPal.color("qgc-red")
            }
            CDFStatCard { 
                label: qsTr("Samples"); 
                value: root.formationResult.sample_count ? root.formationResult.sample_count : "—"
            }
            CDFStatCard { 
                label: qsTr("In Tolerance (<2m)"); 
                value: root.formationResult.in_tolerance ? root.formationResult.in_tolerance + "%" : "—"
                color: (root.formationResult.in_tolerance || 0) > 90 ? qgcPal.color("qgc-green") : qgcPal.color("qgc-orange")
            }
            CDFStatCard { 
                label: qsTr("Degraded (2-5m)"); 
                value: root.formationResult.degraded ? root.formationResult.degraded + "%" : "—"
                color: qgcPal.color("qgc-yellow")
            }
            CDFStatCard { 
                label: qsTr("Lost (>5m)"); 
                value: root.formationResult.lost ? root.formationResult.lost + "%" : "—"
                color: qgcPal.color("qgc-red")
            }
        }
        
        // CDF Chart
        GroupBox {
            Layout.fillWidth: true
            Layout.fillHeight: true
            title: qsTr("Position Error CDF")
            
            ChartView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                antialiasing: true
                theme: ChartView.ChartThemeDark
                animationOptions: ChartView.SeriesAnimations
                
                LineSeries {
                    id: cdfSeries
                    name: qsTr("Position Error CDF")
                    color: qgcPal.color("qgc-blue")
                    useOpenGL: true
                }
                
                // Reference lines
                LineSeries {
                    id: ref2m
                    name: qsTr("2m threshold")
                    color: qgcPal.color("qgc-green")
                    useOpenGL: true
                }
                
                LineSeries {
                    id: ref5m
                    name: qsTr("5m threshold")
                    color: qgcPal.color("qgc-red")
                    useOpenGL: true
                }
                
                ValueAxis {
                    id: axisX
                    min: 0
                    max: 20
                    titleText: qsTr("Position Error (m)")
                    tickCount: 11
                }
                ValueAxis {
                    id: axisY
                    min: 0
                    max: 1
                    titleText: qsTr("Cumulative Probability")
                    labelFormat: "%.0%"
                    tickCount: 11
                }
            }
        }
        
        // Error by vehicle
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Per-Vehicle Formation Error")
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.perVehicleModel()
                
                TableViewColumn {
                    role: "vehicle_id"
                    title: qsTr("Vehicle")
                    width: ScreenTools.defaultFontPixelWidth * 10
                }
                TableViewColumn {
                    role: "callsign"
                    title: qsTr("Callsign")
                    width: ScreenTools.defaultFontPixelWidth * 15
                }
                TableViewColumn {
                    role: "mean_error"
                    title: qsTr("Mean (m)")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { text: display.toFixed(2) }
                }
                TableViewColumn {
                    role: "max_error"
                    title: qsTr("Max (m)")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { text: display.toFixed(2) }
                }
                TableViewColumn {
                    role: "std_error"
                    title: qsTr("Std (m)")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { text: display.toFixed(2) }
                }
                TableViewColumn {
                    role: "p95_error"
                    title: qsTr("P95 (m)")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { text: display.toFixed(2) }
                }
                TableViewColumn {
                    role: "in_tolerance_pct"
                    title: qsTr("In Tol. %")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: display.toFixed(1)
                        color: display > 90 ? qgcPal.color("qgc-green") : display > 70 ? qgcPal.color("qgc-orange") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "lost_pct"
                    title: qsTr("Lost %")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate { 
                        text: display.toFixed(1)
                        color: display > 10 ? qgcPal.color("qgc-red") : qgcPal.color("qgc-green")
                    }
                }
            }
        }
        
        // Error timeline
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Formation Error Timeline")
            
            ChartView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                antialiasing: true
                theme: ChartView.ChartThemeDark
                animationOptions: ChartView.SeriesAnimations
                
                LineSeries {
                    id: posErrorSeries
                    name: qsTr("Position Error (m)")
                    color: qgcPal.color("qgc-red")
                    useOpenGL: true
                }
                
                LineSeries {
                    id: velErrorSeries
                    name: qsTr("Velocity Error (m/s)")
                    color: qgcPal.color("qgc-blue")
                    useOpenGL: true
                }
                
                LineSeries {
                    id: attErrorSeries
                    name: qsTr("Attitude Error (deg)")
                    color: qgcPal.color("qgc-yellow")
                    useOpenGL: true
                }
                
                ValueAxis {
                    id: axisX2
                    min: 0
                    max: 3600
                    titleText: qsTr("Time (s)")
                }
                ValueAxis {
                    id: axisY2
                    min: 0
                    max: 10
                    titleText: qsTr("Error")
                }
            }
        }
    }
    
    // Update CDF chart from result
    function updateCDFChart() {
        cdfSeries.clear()
        ref2m.clear()
        ref5m.clear()
        
        if (formationResult.cdf_x && formationResult.cdf_y) {
            for (var i = 0; i < formationResult.cdf_x.length; i++) {
                cdfSeries.append(formationResult.cdf_x[i], formationResult.cdf_y[i])
            }
        }
        
        // Reference lines at 2m and 5m
        ref2m.append(0, 0)
        ref2m.append(2, 0)
        ref2m.append(2, 1)
        ref2m.append(20, 1)
        
        ref5m.append(0, 0)
        ref5m.append(5, 0)
        ref5m.append(5, 1)
        ref5m.append(20, 1)
    }
    
    // Update timeline chart
    function updateTimelineChart() {
        posErrorSeries.clear()
        velErrorSeries.clear()
        attErrorSeries.clear()
        
        if (formationResult.timestamps_us && formationResult.position_errors_m) {
            var t0 = formationResult.timestamps_us[0]
            for (var i = 0; i < formationResult.timestamps_us.length; i++) {
                var t = (formationResult.timestamps_us[i] - t0) / 1e6
                posErrorSeries.append(t, formationResult.position_errors_m[i])
            }
        }
        if (formationResult.timestamps_us && formationResult.velocity_errors_mps) {
            var t0 = formationResult.timestamps_us[0]
            for (var i = 0; i < formationResult.timestamps_us.length; i++) {
                var t = (formationResult.timestamps_us[i] - t0) / 1e6
                velErrorSeries.append(t, formationResult.velocity_errors_mps[i])
            }
        }
        if (formationResult.timestamps_us && formationResult.attitude_errors_deg) {
            var t0 = formationResult.timestamps_us[0]
            for (var i = 0; i < formationResult.timestamps_us.length; i++) {
                var t = (formationResult.timestamps_us[i] - t0) / 1e6
                attErrorSeries.append(t, formationResult.attitude_errors_deg[i])
            }
        }
    }
    
    // Update on result change
    onFormationResultChanged: {
        updateCDFChart()
        updateTimelineChart()
    }
    
    function perVehicleModel() {
        var model = []
        if (formationResult.per_vehicle) {
            for (var vid in formationResult.per_vehicle) {
                var v = formationResult.per_vehicle[vid]
                model.push({
                    vehicle_id: vid,
                    callsign: v.callsign || ("DRONE" + vid),
                    mean_error: v.mean_error_m || 0,
                    max_error: v.max_error_m || 0,
                    std_error: v.std_error_m || 0,
                    p95_error: v.p95_error_m || 0,
                    in_tolerance_pct: v.in_tolerance_pct || 0,
                    lost_pct: v.lost_pct || 0
                })
            }
        }
        return model
    }
    
    // Reusable stat card
    Component {
        id: CDFStatCard
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