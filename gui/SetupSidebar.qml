import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

/**
 * SetupSidebar.qml - Right sidebar setup assistant panel
 * Displays step info, input controls, advanced settings, progress, and action button
 */
Rectangle {
    id: sidebar
    
    // Properties
    property string stepLabel: "STEP 1: SIMULATOR SETUP"
    property string selectedSimulator: ""
    property string simulationDuration: "60"
    property bool applicationLayerEnabled: false
    property bool carmakerEnabled: false
    property bool advancedExpanded: false
    
    // Signals
    signal doneClicked()
    signal simulatorSelected(string name)
    
    color: "#FAFBFC"
    width: 320
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        anchors.topMargin: 30
        spacing: 24
        
        // ===== STEP LABEL =====
        Text {
            text: sidebar.stepLabel
            font.pixelSize: 11
            font.bold: true
            font.letterSpacing: 1.2
            color: "#8B92A0"
            Layout.fillWidth: true
        }
        
        // ===== SIMULATION DURATION INPUT =====
        ColumnLayout {
            spacing: 8
            Layout.fillWidth: true
            
            Text {
                text: "Simulation Duration"
                font.pixelSize: 12
                font.bold: true
                color: "#4B5563"
            }
            
            RowLayout {
                spacing: 8
                Layout.fillWidth: true
                
                TextField {
                    text: sidebar.simulationDuration
                    font.pixelSize: 12
                    color: "#1F2937"
                    Layout.preferredWidth: 80
                    Layout.preferredHeight: 36
                    horizontalAlignment: TextInput.AlignHCenter
                    padding: 8
                    onEditingFinished: sidebar.simulationDuration = text
                }
                
                Text {
                    text: "seconds"
                    font.pixelSize: 12
                    color: "#6B7280"
                    Layout.alignment: Qt.AlignVCenter
                }
                
                Item { Layout.fillWidth: true }
            }
        }
        
        // ===== ADVANCED SETTINGS COLLAPSIBLE =====
        ColumnLayout {
            spacing: 12
            Layout.fillWidth: true
            
            // Header
            MouseArea {
                Layout.fillWidth: true
                Layout.preferredHeight: 36
                hoverEnabled: true
                onClicked: sidebar.advancedExpanded = !sidebar.advancedExpanded
                
                RowLayout {
                    anchors.fill: parent
                    spacing: 10
                    
                    Text {
                        text: "⚙"
                        font.pixelSize: 14
                        color: "#6B7280"
                    }
                    
                    Text {
                        text: "Advanced Settings"
                        font.pixelSize: 12
                        font.bold: true
                        color: "#4B5563"
                        Layout.fillWidth: true
                    }
                    
                    Text {
                        text: sidebar.advancedExpanded ? "▼" : "▶"
                        font.pixelSize: 12
                        color: "#9CA3AF"
                        Behavior on rotation {
                            NumberAnimation { duration: 200 }
                        }
                    }
                }
                
                Rectangle {
                    anchors.fill: parent
                    color: parent.containsMouse ? "#F3F4F6" : "transparent"
                    radius: 6
                    z: -1
                    Behavior on color { ColorAnimation { duration: 150 } }
                }
            }
            
            // Hidden content
            Item {
                Layout.fillWidth: true
                height: sidebar.advancedExpanded ? 60 : 0
                clip: true
                visible: height > 0
                
                Behavior on height {
                    NumberAnimation { duration: 250; easing.type: Easing.OutCubic }
                }
                
                ColumnLayout {
                    width: parent.width
                    anchors.top: parent.top
                    spacing: 8
                    
                    CheckBox {
                        text: "Verbose Logging"
                        font.pixelSize: 11
                        Layout.fillWidth: true
                    }
                    
                    CheckBox {
                        text: "Debug Mode"
                        font.pixelSize: 11
                        Layout.fillWidth: true
                    }
                }
            }
        }
        
        // ===== PROGRESS / STATUS SUMMARY =====
        ColumnLayout {
            spacing: 10
            Layout.fillWidth: true
            
            Text {
                text: "Progress"
                font.pixelSize: 11
                font.bold: true
                color: "#8B92A0"
                font.letterSpacing: 1.0
            }
            
            StatusRow {
                label: "Selected Simulator: " + (sidebar.selectedSimulator || "None")
                isComplete: sidebar.selectedSimulator !== ""
            }
            
            StatusRow {
                label: "Application Layer"
                isComplete: sidebar.applicationLayerEnabled
            }
            
            StatusRow {
                label: "IPG CarMaker"
                isComplete: sidebar.carmakerEnabled
            }
        }
        
        // Spacer
        Item { Layout.fillHeight: true }
        
        // ===== BOTTOM PRIMARY BUTTON =====
        Button {
            text: "Done"
            Layout.fillWidth: true
            Layout.preferredHeight: 44
            font.bold: true
            font.pixelSize: 13
            
            onClicked: sidebar.doneClicked()
        }
    }
}
