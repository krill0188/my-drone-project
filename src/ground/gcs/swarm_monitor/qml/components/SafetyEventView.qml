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
    property var safetyEventsModel: []
    property var filteredEvents: []
    property string severityFilter: "ALL"  // ALL, CRITICAL, ERROR, WARN, INFO
    property string sourceFilter: "ALL"
    property string typeFilter: "ALL"
    
    signal acknowledgeEvent(uint64_t timestamp, uint8_t ackBy)
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 2
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Filter toolbar
        ToolBar {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 3
            
            Label { text: qsTr("Severity:"); Layout.alignment: Qt.AlignVCenter }
            ComboBox {
                id: severityCombo
                model: ["ALL", "CRITICAL", "ERROR", "WARN", "INFO"]
                currentText: root.severityFilter
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 12
                onActivated: root.severityFilter = model[index]
            }
            
            Label { text: qsTr("Source:"); Layout.alignment: Qt.AlignVCenter }
            ComboBox {
                id: sourceCombo
                model: ["ALL"].concat(root.uniqueSources())
                currentText: root.sourceFilter
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 12
                onActivated: root.sourceFilter = model[index]
            }
            
            Label { text: qsTr("Type:"); Layout.alignment: Qt.AlignVCenter }
            ComboBox {
                id: typeCombo
                model: ["ALL"].concat(root.uniqueTypes())
                currentText: root.typeFilter
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 12
                onActivated: root.typeFilter = model[index]
            }
            
            Item { Layout.fillWidth: true }
            
            ToolButton {
                text: qsTr("Acknowledge All Visible")
                enabled: root.hasUnacknowledged()
                onClicked: {
                    for (var i = 0; i < filteredEvents.length; i++) {
                        if (!filteredEvents[i].acknowledged && monitor) {
                            monitor.acknowledgeSafetyEvent(filteredEvents[i].timestamp_us, 255)  // 255 = GCS
                        }
                    }
                }
            }
            
            ToolButton {
                text: qsTr("Clear Acknowledged")
                onClicked: {
                    // Would clear from local model
                }
            }
            
            ToolButton {
                text: qsTr("Export")
                onClicked: {
                    // Export to CSV
                }
            }
        }
        
        // Stats cards
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 5
            spacing: ScreenTools.defaultFontPixelWidth * 2
            
            EventStatCard { label: qsTr("Total"); value: safetyEventsModel.length; color: qgcPal.color("qgc-primary") }
            EventStatCard { label: qsTr("Unacked"); value: root.unacknowledgedCount(); color: qgcPal.color("qgc-red") }
            EventStatCard { label: qsTr("Critical"); value: root.severityCount("CRITICAL"); color: qgcPal.color("qgc-red") }
            EventStatCard { label: qsTr("Error"); value: root.severityCount("ERROR"); color: qgcPal.color("qgc-orange") }
            EventStatCard { label: qsTr("Warning"); value: root.severityCount("WARN"); color: qgcPal.color("qgc-yellow") }
            EventStatCard { label: qsTr("Info"); value: root.severityCount("INFO"); color: qgcPal.color("qgc-blue") }
        }
        
        // Events table
        GroupBox {
            Layout.fillWidth: true
            Layout.fillHeight: true
            title: qsTr("Safety Events (%1)").arg(filteredEvents.length)
            
            TableView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.filteredEvents
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
                    role: "source_id"
                    title: qsTr("Src")
                    width: ScreenTools.defaultFontPixelWidth * 6
                }
                TableViewColumn {
                    role: "event_type"
                    title: qsTr("Type")
                    width: ScreenTools.defaultFontPixelWidth * 14
                }
                TableViewColumn {
                    role: "severity"
                    title: qsTr("Severity")
                    width: ScreenTools.defaultFontPixelWidth * 10
                    delegate: ItemDelegate { 
                        text: display
                        color: display === "CRITICAL" ? qgcPal.color("qgc-red") : 
                               display === "ERROR" ? qgcPal.color("qgc-orange") :
                               display === "WARN" ? qgcPal.color("qgc-yellow") : qgcPal.color("qgc-blue")
                        font.bold: display === "CRITICAL" || display === "ERROR"
                    }
                }
                TableViewColumn {
                    role: "description"
                    title: qsTr("Description")
                    width: ScreenTools.defaultFontPixelWidth * 30
                    delegate: ItemDelegate { 
                        text: display
                        wrapMode: Text.WordWrap
                    }
                }
                TableViewColumn {
                    role: "acknowledged"
                    title: qsTr("Acked")
                    width: ScreenTools.defaultFontPixelWidth * 8
                    delegate: ItemDelegate { 
                        text: display ? qsTr("✓") : qsTr("✗")
                        color: display ? qgcPal.color("qgc-green") : qgcPal.color("qgc-red")
                    }
                }
                TableViewColumn {
                    role: "acknowledged_by"
                    title: qsTr("By")
                    width: ScreenTools.defaultFontPixelWidth * 6
                    delegate: ItemDelegate { text: display ? "#" + display : "—" }
                }
                TableViewColumn {
                    role: ""
                    title: qsTr("Action")
                    width: ScreenTools.defaultFontPixelWidth * 12
                    delegate: ItemDelegate {
                        ToolButton {
                            text: display.acknowledged ? qsTr("Acked") : qsTr("Ack")
                            enabled: !display.acknowledged
                            onClicked: {
                                if (monitor) {
                                    acknowledgeEvent(display.timestamp_us, 255)
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Event timeline chart
        GroupBox {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 15
            title: qsTr("Event Timeline (Last Hour)")
            
            ChartView {
                anchors.fill: parent
                antialiasing: true
                theme: ChartView.ChartThemeDark
                animationOptions: ChartView.SeriesAnimations
                
                ScatterSeries {
                    id: criticalSeries
                    name: qsTr("Critical")
                    color: qgcPal.color("qgc-red")
                    markerSize: 10
                }
                
                ScatterSeries {
                    id: errorSeries
                    name: qsTr("Error")
                    color: qgcPal.color("qgc-orange")
                    markerSize: 8
                }
                
                ScatterSeries {
                    id: warnSeries
                    name: qsTr("Warning")
                    color: qgcPal.color("qgc-yellow")
                    markerSize: 6
                }
                
                ScatterSeries {
                    id: infoSeries
                    name: qsTr("Info")
                    color: qgcPal.color("qgc-blue")
                    markerSize: 5
                }
                
                ValueAxis {
                    id: axisX
                    min: 0
                    max: 3600  // 1 hour in seconds
                    labelFormat: "%ds ago"
                    titleText: qsTr("Seconds Ago")
                    reverse: true
                }
                ValueAxis {
                    id: axisY
                    min: 0
                    max: 4
                    titleText: qsTr("Severity")
                }
            }
        }
    }
    
    // Filter events
    function updateFilteredEvents() {
        filteredEvents = safetyEventsModel.filter(function(e) {
            if (severityFilter !== "ALL" && e.severity !== severityFilter) return false
            if (sourceFilter !== "ALL" && e.source_id !== parseInt(sourceFilter)) return false
            if (typeFilter !== "ALL" && e.event_type !== typeFilter) return false
            return true
        })
    }
    
    // Update when model or filters change
    onSafetyEventsModelChanged: updateFilteredEvents()
    onSeverityFilterChanged: updateFilteredEvents()
    onSourceFilterChanged: updateFilteredEvents()
    onTypeFilterChanged: updateFilteredEvents()
    
    // Update timeline chart
    Timer {
        interval: 5000
        running: true
        repeat: true
        onTriggered: {
            var now = Date.now() / 1000
            criticalSeries.clear()
            errorSeries.clear()
            warnSeries.clear()
            infoSeries.clear()
            
            for (var i = 0; i < safetyEventsModel.length; i++) {
                var e = safetyEventsModel[i]
                var age = now - e.timestamp_us / 1e6
                if (age < 0 || age > 3600) continue
                
                var y = e.severity === "CRITICAL" ? 3 : 
                        e.severity === "ERROR" ? 2 : 
                        e.severity === "WARN" ? 1 : 0
                
                if (e.severity === "CRITICAL") criticalSeries.append(age, y)
                else if (e.severity === "ERROR") errorSeries.append(age, y)
                else if (e.severity === "WARN") warnSeries.append(age, y)
                else infoSeries.append(age, y)
            }
        }
    }
    
    // Helper functions
    function uniqueSources() {
        var sources = []
        for (var i = 0; i < safetyEventsModel.length; i++) {
            var id = safetyEventsModel[i].source_id.toString()
            if (sources.indexOf(id) === -1) sources.push(id)
        }
        return sources.sort()
    }
    
    function uniqueTypes() {
        var types = []
        for (var i = 0; i < safetyEventsModel.length; i++) {
            var t = safetyEventsModel[i].event_type
            if (types.indexOf(t) === -1) types.push(t)
        }
        return types.sort()
    }
    
    function severityCount(severity) {
        var count = 0
        for (var i = 0; i < safetyEventsModel.length; i++) {
            if (safetyEventsModel[i].severity === severity) count++
        }
        return count
    }
    
    function unacknowledgedCount() {
        var count = 0
        for (var i = 0; i < safetyEventsModel.length; i++) {
            if (!safetyEventsModel[i].acknowledged) count++
        }
        return count
    }
    
    function hasUnacknowledged() {
        for (var i = 0; i < filteredEvents.length; i++) {
            if (!filteredEvents[i].acknowledged) return true
        }
        return false
    }
    
    // Reusable stat card
    Component {
        id: EventStatCard
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: ScreenTools.defaultFontPixelWidth * 2
            color: qgcPal.window
            border.color: parent.color
            border.width: 2
            
            property string label: ""
            property int value: 0
            property color color: qgcPal.color("qgc-primary")
            
            Column {
                anchors.centerIn: parent
                spacing: ScreenTools.defaultFontPixelWidth
                
                Text {
                    text: value
                    font.pixelSize: ScreenTools.defaultFontPixelSize * 2
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