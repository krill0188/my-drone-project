import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtPositioning 5.15
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.FactSystem 1.0
import QGroundControl.FlightMap 1.0
import QGroundControl.ScreenTools 1.0
import QGroundControl.Palette 1.0
import SwarmMissionPlanner 1.0

Item {
    id: root
    property SwarmMissionPlanner planner: null
    property string activeMissionId: ""
    property var currentMission: null
    property var vehicleList: []
    
    signal missionChanged(string missionId)
    signal requestMapCenter(qreal lat, qreal lon)
    
    function loadMission(missionId) {
        if (!planner) return
        var result = planner.loadMission(missionId)
        if (result) {
            activeMissionId = missionId
            currentMission = planner.getMission(missionId)
            missionChanged(missionId)
        }
    }
    
    function createNewMission(name, description) {
        if (!planner) return ""
        var missionId = planner.createMission(name, description)
        loadMission(missionId)
        return missionId
    }
    
    // Main layout
    ColumnLayout {
        anchors.fill: parent
        spacing: ScreenTools.defaultFontPixelWidth * 2
        
        // Toolbar
        ToolBar {
            Layout.fillWidth: true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 3
            
            ToolButton {
                text: qsTr("New Mission")
                iconSource: "qrc:/qmlImages/fileNew.svg"
                onClicked: {
                    var name = prompt(qsTr("Mission name:"), "New Swarm Mission")
                    if (name) {
                        var desc = prompt(qsTr("Description (optional):"), "")
                        createNewMission(name, desc)
                    }
                }
            }
            
            ToolButton {
                text: qsTr("Save")
                iconSource: "qrc:/qmlImages/save.svg"
                enabled: activeMissionId !== ""
                onClicked: {
                    if (activeMissionId) planner.saveMission(activeMissionId)
                }
            }
            
            ToolButton {
                text: qsTr("Load")
                iconSource: "qrc:/qmlImages/folderOpen.svg"
                onClicked: missionListDialog.open()
            }
            
            ToolButton {
                text: qsTr("Delete")
                iconSource: "qrc:/qmlImages/trash.svg"
                enabled: activeMissionId !== ""
                onClicked: {
                    if (activeMissionId && confirm(qsTr("Delete mission?"))) {
                        planner.deleteMission(activeMissionId)
                        activeMissionId = ""
                        currentMission = null
                    }
                }
            }
            
            Item { Layout.fillWidth: true }
            
            ComboBox {
                id: droneSelector
                model: vehicleList
                textRole: "displayName"
                valueRole: "id"
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 20
                onActivated: {
                    if (activeMissionId) {
                        var mission = currentMission
                        var drones = mission.participating_drones
                        var id = model[index].id
                        if (drones.indexOf(id) === -1) {
                            drones.push(id)
                            planner.saveMission(activeMissionId)
                            currentMission = planner.getMission(activeMissionId)
                        }
                    }
                }
            }
            
            ToolButton {
                text: qsTr("Add Drone")
                enabled: droneSelector.count > 0
                onClicked: {
                    if (activeMissionId && droneSelector.currentIndex >= 0) {
                        var id = droneSelector.model[droneSelector.currentIndex].id
                        var mission = currentMission
                        if (mission.participating_drones.indexOf(id) === -1) {
                            mission.participating_drones.push(id)
                            planner.saveMission(activeMissionId)
                            currentMission = planner.getMission(activeMissionId)
                        }
                    }
                }
            }
        }
        
        // Main content area
        SplitView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            // Left panel - Mission tree & editors
            Rectangle {
                Layout.minimumWidth: ScreenTools.defaultFontPixelWidth * 40
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 50
                Layout.fillHeight: true
                color: qgcPal.window
                
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                    spacing: ScreenTools.defaultFontPixelWidth * 2
                    
                    // Mission info
                    GroupBox {
                        Layout.fillWidth: true
                        title: qsTr("Mission Info")
                        
                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                            spacing: ScreenTools.defaultFontPixelWidth
                            
                            TextField {
                                Layout.fillWidth: true
                                placeholderText: qsTr("Mission name")
                                text: currentMission ? currentMission.name : ""
                                onTextChanged: {
                                    if (activeMissionId && currentMission) {
                                        currentMission.name = text
                                        planner.saveMission(activeMissionId)
                                    }
                                }
                            }
                            
                            TextArea {
                                Layout.fillWidth: true
                                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 4
                                placeholderText: qsTr("Description")
                                text: currentMission ? currentMission.description : ""
                                onTextChanged: {
                                    if (activeMissionId && currentMission) {
                                        currentMission.description = text
                                        planner.saveMission(activeMissionId)
                                    }
                                }
                            }
                            
                            RowLayout {
                                Label { text: qsTr("Strategy:") }
                                ComboBox {
                                    Layout.fillWidth: true
                                    model: ["CBBA", "Greedy", "Manual"]
                                    currentText: currentMission ? currentMission.allocation_strategy : "CBBA"
                                    onActivated: {
                                        if (activeMissionId && currentMission) {
                                            currentMission.allocation_strategy = model[index]
                                            planner.saveMission(activeMissionId)
                                        }
                                    }
                                }
                            }
                            
                            RowLayout {
                                Label { text: qsTr("Auto Allocate:") }
                                Switch {
                                    checked: currentMission ? currentMission.auto_allocate : true
                                    onToggled: {
                                        if (activeMissionId && currentMission) {
                                            currentMission.auto_allocate = checked
                                            planner.saveMission(activeMissionId)
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    // Waypoints list
                    GroupBox {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        title: qsTr("Waypoints (%1)").arg(currentMission ? currentMission.waypoints.length : 0)
                        
                        ListView {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            model: currentMission ? currentMission.waypoints : []
                            delegate: WaypointDelegate {}
                            clip: true
                        }
                        
                        RowLayout {
                            ToolButton {
                                text: qsTr("Add WP")
                                iconSource: "qrc:/qmlIcons/mapMarkerPlus.svg"
                                onClicked: addWaypointDialog.open()
                            }
                            ToolButton {
                                text: qsTr("Remove")
                                iconSource: "qrc:/qmlIcons/trash.svg"
                                enabled: waypointListView.currentIndex >= 0
                                onClicked: {
                                    if (activeMissionId && waypointListView.currentIndex >= 0) {
                                        var wp = currentMission.waypoints[waypointListView.currentIndex]
                                        planner.removeWaypoint(activeMissionId, wp.sequence.toString())
                                        currentMission = planner.getMission(activeMissionId)
                                    }
                                }
                            }
                        }
                    }
                    
                    // Search polygons
                    GroupBox {
                        Layout.fillWidth: true
                        title: qsTr("Search Polygons (%1)").arg(currentMission ? currentMission.search_polygons.length : 0)
                        Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 8
                        
                        ListView {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            model: currentMission ? currentMission.search_polygons : []
                            delegate: SearchPolygonDelegate {}
                            clip: true
                        }
                        
                        RowLayout {
                            ToolButton {
                                text: qsTr("Add Polygon")
                                iconSource: "qrc:/qmlIcons/mapPolygon.svg"
                                onClicked: addSearchPolygonDialog.open()
                            }
                        }
                    }
                    
                    // Inspect targets
                    GroupBox {
                        Layout.fillWidth: true
                        title: qsTr("Inspect Targets (%1)").arg(currentMission ? currentMission.inspect_targets.length : 0)
                        Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 8
                        
                        ListView {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            model: currentMission ? currentMission.inspect_targets : []
                            delegate: InspectTargetDelegate {}
                            clip: true
                        }
                        
                        RowLayout {
                            ToolButton {
                                text: qsTr("Add Target")
                                iconSource: "qrc:/qmlIcons/target.svg"
                                onClicked: addInspectTargetDialog.open()
                            }
                        }
                    }
                }
            }
            
            // Right panel - Map view & allocation
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: qgcPal.window
                
                ColumnLayout {
                    anchors.fill: parent
                    spacing: ScreenTools.defaultFontPixelWidth * 2
                    
                    // Map view
                    FlightMap {
                        id: flightMap
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: ScreenTools.defaultFontPixelHeight * 20
                        
                        // Mission items would be shown here
                    }
                    
                    // CBBA Allocation controls
                    GroupBox {
                        Layout.fillWidth: true
                        title: qsTr("CBBA Allocation")
                        Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 12
                        
                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                            spacing: ScreenTools.defaultFontPixelWidth
                            
                            RowLayout {
                                ToolButton {
                                    text: qsTr("Preview Allocation")
                                    Layout.fillWidth: true
                                    enabled: activeMissionId && currentMission && currentMission.participating_drones.length > 0
                                    onClicked: {
                                        var result = planner.previewAllocation(activeMissionId)
                                        allocationResult = result
                                        allocationPreviewDialog.open()
                                    }
                                }
                                ToolButton {
                                    text: qsTr("Run CBBA")
                                    Layout.fillWidth: true
                                    enabled: activeMissionId && currentMission && currentMission.participating_drones.length > 0
                                    onClicked: {
                                        allocationProgressBar.visible = true
                                        planner.runCBBAAllocation(activeMissionId)
                                    }
                                }
                            }
                            
                            ProgressBar {
                                id: allocationProgressBar
                                Layout.fillWidth: true
                                visible: false
                                from: 0
                                to: 100
                                value: 0
                            }
                            
                            Label {
                                id: allocationStatusLabel
                                Layout.fillWidth: true
                                text: qsTr("Ready")
                                wrapMode: Text.WordWrap
                            }
                            
                            // Allocation result display
                            ListView {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                model: allocationResult ? allocationResult.assignments : []
                                delegate: AllocationDelegate {}
                                clip: true
                            }
                        }
                    }
                    
                    // Upload/Execute controls
                    GroupBox {
                        Layout.fillWidth: true
                        title: qsTr("Mission Execution")
                        Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 8
                        
                        RowLayout {
                            anchors.fill: parent
                            anchors.margins: ScreenTools.defaultFontPixelWidth * 2
                            spacing: ScreenTools.defaultFontPixelWidth
                            
                            ToolButton {
                                text: qsTr("Upload All")
                                Layout.fillWidth: true
                                enabled: activeMissionId && currentMission && currentMission.participating_drones.length > 0
                                onClicked: planner.uploadMission(activeMissionId)
                            }
                            
                            ToolButton {
                                text: qsTr("Start")
                                Layout.fillWidth: true
                                enabled: activeMissionId && currentMission && currentMission.state === 3  // Ready
                                onClicked: planner.startMission(activeMissionId)
                            }
                            
                            ToolButton {
                                text: qsTr("Pause")
                                Layout.fillWidth: true
                                enabled: activeMissionId && currentMission && currentMission.state === 4  // Active
                                onClicked: planner.pauseMission(activeMissionId)
                            }
                            
                            ToolButton {
                                text: qsTr("Abort")
                                Layout.fillWidth: true
                                enabled: activeMissionId && currentMission && (currentMission.state === 4 || currentMission.state === 5)  // Active or Paused
                                onClicked: planner.abortMission(activeMissionId)
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Dialogs
    Dialog {
        id: addWaypointDialog
        title: qsTr("Add Waypoint")
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: {
            if (activeMissionId) {
                var data = {
                    "lat": wpLat.value,
                    "lon": wpLon.value,
                    "alt": wpAlt.value,
                    "command": wpCommand.currentIndex === 0 ? 16 : 21,  // WAYPOINT or LOITER
                    "task_type": "waypoint",
                    "task_id": ""
                }
                planner.addWaypoint(activeMissionId, data)
                currentMission = planner.getMission(activeMissionId)
            }
        }
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: ScreenTools.defaultFontPixelWidth * 2
            spacing: ScreenTools.defaultFontPixelWidth
            
            RowLayout {
                Label { text: qsTr("Latitude:") }
                SpinBox { id: wpLat; decimals: 7; minimum: -90; maximum: 90; value: 0; Layout.fillWidth: true }
            }
            RowLayout {
                Label { text: qsTr("Longitude:") }
                SpinBox { id: wpLon; decimals: 7; minimum: -180; maximum: 180; value: 0; Layout.fillWidth: true }
            }
            RowLayout {
                Label { text: qsTr("Altitude (m):") }
                SpinBox { id: wpAlt; decimals: 1; minimum: 0; maximum: 10000; value: 50; Layout.fillWidth: true }
            }
            RowLayout {
                Label { text: qsTr("Command:") }
                ComboBox { id: wpCommand; model: ["WAYPOINT", "LOITER", "TAKEOFF", "LAND"]; Layout.fillWidth: true }
            }
        }
    }
    
    Dialog {
        id: addSearchPolygonDialog
        title: qsTr("Add Search Polygon")
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: {
            if (activeMissionId) {
                var data = {
                    "name": spName.text,
                    "vertices": spVerticesModel.items,  // Would be populated from map
                    "search_altitude_m": spAlt.value,
                    "spacing_m": spSpacing.value,
                    "angle_deg": spAngle.value,
                    "pattern_type": spPattern.currentText
                }
                planner.addSearchPolygon(activeMissionId, data)
                currentMission = planner.getMission(activeMissionId)
            }
        }
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: ScreenTools.defaultFontPixelWidth * 2
            spacing: ScreenTools.defaultFontPixelWidth
            
            RowLayout {
                Label { text: qsTr("Name:") }
                TextField { id: spName; placeholderText: "Search Area 1"; Layout.fillWidth: true }
            }
            RowLayout {
                Label { text: qsTr("Altitude (m):") }
                SpinBox { id: spAlt; decimals: 1; minimum: 10; maximum: 1000; value: 50; Layout.fillWidth: true }
            }
            RowLayout {
                Label { text: qsTr("Spacing (m):") }
                SpinBox { id: spSpacing; decimals: 1; minimum: 5; maximum: 500; value: 20; Layout.fillWidth: true }
            }
            RowLayout {
                Label { text: qsTr("Angle (deg):") }
                SpinBox { id: spAngle; decimals: 1; minimum: -180; maximum: 180; value: 0; Layout.fillWidth: true }
            }
            RowLayout {
                Label { text: qsTr("Pattern:") }
                ComboBox { id: spPattern; model: ["grid", "spiral", "creeping_line"]; Layout.fillWidth: true }
            }
            Label { text: qsTr("Vertices: Click on map to define polygon area") }
        }
    }
    
    Dialog {
        id: addInspectTargetDialog
        title: qsTr("Add Inspect Target")
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel
        onAccepted: {
            if (activeMissionId) {
                var data = {
                    "name": itName.text,
                    "lat": itLat.value,
                    "lon": itLon.value,
                    "inspect_altitude_m": itAlt.value,
                    "standoff_distance_m": itStandoff.value,
                    "yaw_angles_deg": itYawModel.items,
                    "gimbal_pitch_deg": itPitch.value,
                    "target_type": itType.currentText,
                    "priority": itPriority.currentText
                }
                planner.addInspectTarget(activeMissionId, data)
                currentMission = planner.getMission(activeMissionId)
            }
        }
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: ScreenTools.defaultFontPixelWidth * 2
            spacing: ScreenTools.defaultFontPixelWidth
            
            RowLayout { Label { text: qsTr("Name:") }; TextField { id: itName; Layout.fillWidth: true } }
            RowLayout { Label { text: qsTr("Latitude:") }; SpinBox { id: itLat; decimals: 7; minimum: -90; maximum: 90; Layout.fillWidth: true } }
            RowLayout { Label { text: qsTr("Longitude:") }; SpinBox { id: itLon; decimals: 7; minimum: -180; maximum: 180; Layout.fillWidth: true } }
            RowLayout { Label { text: qsTr("Inspect Alt (m):") }; SpinBox { id: itAlt; decimals: 1; minimum: 1; maximum: 100; value: 10; Layout.fillWidth: true } }
            RowLayout { Label { text: qsTr("Standoff (m):") }; SpinBox { id: itStandoff; decimals: 1; minimum: 1; maximum: 50; value: 5; Layout.fillWidth: true } }
            RowLayout { Label { text: qsTr("Gimbal Pitch:") }; SpinBox { id: itPitch; decimals: 1; minimum: -90; maximum: 90; value: -90; Layout.fillWidth: true } }
            RowLayout { Label { text: qsTr("Yaw Angles:") }; TextField { id: itYawInput; placeholderText: "0, 90, 180, 270"; Layout.fillWidth: true } }
            RowLayout { Label { text: qsTr("Target Type:") }; ComboBox { id: itType; model: ["structure", "tower", "bridge", "solar_panel", "custom"]; Layout.fillWidth: true } }
            RowLayout { Label { text: qsTr("Priority:") }; ComboBox { id: itPriority; model: ["high", "medium", "low"]; Layout.fillWidth: true } }
        }
    }
    
    Dialog {
        id: missionListDialog
        title: qsTr("Load Mission")
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel
        
        ListView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            model: planner.listMissions()
            delegate: ItemDelegate {
                text: model.name + " (" + model.waypoint_count + " WP, " + model.drone_count + " drones)"
                width: ListView.view.width
                onClicked: {
                    ListView.view.currentIndex = index
                }
            }
        }
        
        onAccepted: {
            if (currentIndex >= 0) {
                var mission = listMissionsModel.get(currentIndex)
                loadMission(mission.id)
            }
        }
    }
    
    Dialog {
        id: allocationPreviewDialog
        title: qsTr("Allocation Preview")
        modal: true
        standardButtons: Dialog.Apply | Dialog.Cancel
        onAccepted: {
            if (activeMissionId && allocationResult) {
                planner.applyAllocation(activeMissionId, allocationResult)
                currentMission = planner.getMission(activeMissionId)
            }
        }
        
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            minimumWidth: ScreenTools.defaultFontPixelWidth * 60
            minimumHeight: ScreenTools.defaultFontPixelHeight * 30
            
            Label {
                text: qsTr("Drone Assignments:")
                font.bold: true
            }
            
            ListView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                model: allocationResult ? Object.keys(allocationResult.assignments) : []
                delegate: ItemDelegate {
                    text: "Drone " + modelData + ": " + (allocationResult.assignments[modelData].length || 0) + " tasks"
                    width: ListView.view.width
                }
            }
        }
    }
    
    // Connections for signals
    Connections {
        target: planner
        onAllocationProgress: {
            allocationProgressBar.value = progress
            allocationStatusLabel.text = message
        }
        onAllocationCompleted: {
            allocationProgressBar.visible = false
            allocationStatusLabel.text = qsTr("Allocation complete")
            allocationResult = result
        }
        onUploadProgress: {
            allocationStatusLabel.text = qsTr("Upload to drone %1: %2% - %3").arg(droneId).arg(progress).arg(message)
        }
        onMissionStateChanged: {
            currentMission = planner.getMission(activeMissionId)
        }
        onPythonBackendStatusChanged: {
            pythonStatusLabel.text = available ? qsTr("Python backend: Connected") : qsTr("Python backend: Disconnected")
        }
        onErrorOccurred: {
            console.error("Swarm Planner Error:", message)
        }
    }
    
    // List models
    ListModel { id: listMissionsModel }
    ListModel { id: spVerticesModel }
    ListModel { id: itYawModel }
    ListModel { id: waypointListView }
    
    // State
    property var allocationResult: null
    property var pythonStatusLabel: null
    
    Component.onCompleted: {
        // Load vehicle list
        if (planner) {
            // Would connect to VehicleManager to get vehicle list
        }
    }
}

// Delegate components
Component {
    id: WaypointDelegate
    ItemDelegate {
        width: ListView.view.width
        text: "#" + sequence + " [" + lat.toFixed(6) + ", " + lon.toFixed(6) + "] " + alt + "m"
        highlighted: ListView.isCurrentItem
        onClicked: {
            ListView.view.currentIndex = index
            requestMapCenter(lat, lon)
        }
    }
}

Component {
    id: SearchPolygonDelegate
    ItemDelegate {
        width: ListView.view.width
        text: name + " (" + vertices.length + " vertices, " + spacing_m + "m spacing)"
        highlighted: ListView.isCurrentItem
    }
}

Component {
    id: InspectTargetDelegate
    ItemDelegate {
        width: ListView.view.width
        text: name + " @ [" + lat.toFixed(6) + ", " + lon.toFixed(6) + "] " + inspect_altitude_m + "m"
        highlighted: ListView.isCurrentItem
    }
}

Component {
    id: AllocationDelegate
    ItemDelegate {
        width: ListView.view.width
        text: "Drone " + modelData + ": " + (allocationResult.assignments[modelData].length || 0) + " tasks"
    }
}