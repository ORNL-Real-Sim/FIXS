import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

/**
 * SimulationSetupPanel.qml - Modal panel for simulation configuration
 * Provides settings for simulator, runtime, connection details, and message field configuration
 */
Popup {
    id: simulationSetupPanel
    modal: true
    focus: true
    closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
    
    // Signals
    signal setupComplete()
    signal setupCancelled()
    
    // Properties
    property string selectedSimulator: "SUMO"
    property real runtimeSeconds: 120
    property string trafficSimulatorIp: "127.0.0.1"
    property string trafficSimulatorPort: "60002"
    property bool enableRealSim: true
    property string vehicleMessageFieldsMode: "MinimalRequired"
    property var selectedFields: ["id", "speed", "speedDesired"]
    
    width: 500
    height: 700
    anchors.centerIn: Overlay.overlay
    
    background: Rectangle {
        color: "#FFFFFF"
        radius: 12
        border.color: "#E5E7EB"
        border.width: 1
    }
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 24
        spacing: 0
        
        // Header
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 40
            spacing: 12
            
            Text {
                text: "Simulation Setup"
                font.pixelSize: 18
                font.bold: true
                color: "#1F2937"
                Layout.fillWidth: true
            }
            
            Button {
                text: "✕"
                font.pixelSize: 14
                Layout.preferredWidth: 36
                Layout.preferredHeight: 36
                flat: true
                onClicked: {
                    setupCancelled()
                    simulationSetupPanel.close()
                }
            }
        }
        
        // Divider
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 1
            color: "#E5E7EB"
            Layout.topMargin: 12
            Layout.bottomMargin: 16
        }
        
        // Content area
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            ColumnLayout {
                width: simulationSetupPanel.width - 48
                spacing: 16
                
                // Configuration section header
                Text {
                    text: "CONFIGURATION"
                    font.pixelSize: 12
                    font.bold: true
                    color: "#9CA3AF"
                }
                
                // Simulator dropdown
                ColumnLayout {
                    spacing: 6
                    Layout.fillWidth: true
                    
                    Text {
                        text: "Traffic Simulator"
                        font.pixelSize: 12
                        font.bold: true
                        color: "#374151"
                    }
                    
                    ComboBox {
                        model: ["SUMO", "CARLA", "VISSIM"]
                        currentIndex: 0
                        Layout.fillWidth: true
                        Layout.preferredHeight: 36
                        onCurrentTextChanged: simulationSetupPanel.selectedSimulator = currentText
                    }
                }
                
                // Runtime
                ColumnLayout {
                    spacing: 6
                    Layout.fillWidth: true
                    
                    Text {
                        text: "Runtime (seconds)"
                        font.pixelSize: 12
                        font.bold: true
                        color: "#374151"
                    }
                    
                    TextField {
                        text: simulationSetupPanel.runtimeSeconds.toString()
                        font.pixelSize: 12
                        Layout.fillWidth: true
                        Layout.preferredHeight: 36
                        placeholderText: "e.g., 120"
                        onEditingFinished: simulationSetupPanel.runtimeSeconds = parseFloat(text) || 120
                    }
                }
                
                // Divider
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 1
                    color: "#E5E7EB"
                    Layout.topMargin: 8
                    Layout.bottomMargin: 8
                }
                
                // Connection section header
                Text {
                    text: "TRAFFIC SIMULATOR CONNECTION"
                    font.pixelSize: 12
                    font.bold: true
                    color: "#9CA3AF"
                }
                
                // IP and Port row
                RowLayout {
                    spacing: 12
                    Layout.fillWidth: true
                    
                    ColumnLayout {
                        spacing: 6
                        Layout.fillWidth: true
                        
                        Text {
                            text: "IP Address"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#374151"
                        }
                        
                        TextField {
                            text: simulationSetupPanel.trafficSimulatorIp
                            font.pixelSize: 12
                            Layout.fillWidth: true
                            Layout.preferredHeight: 36
                            placeholderText: "127.0.0.1"
                            onEditingFinished: simulationSetupPanel.trafficSimulatorIp = text
                        }
                    }
                    
                    ColumnLayout {
                        spacing: 6
                        Layout.fillWidth: true
                        
                        Text {
                            text: "Port"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#374151"
                        }
                        
                        TextField {
                            text: simulationSetupPanel.trafficSimulatorPort
                            font.pixelSize: 12
                            Layout.fillWidth: true
                            Layout.preferredHeight: 36
                            placeholderText: "60002"
                            onEditingFinished: simulationSetupPanel.trafficSimulatorPort = text
                        }
                    }
                }
                
                // Enable Real Sim toggle
                RowLayout {
                    spacing: 12
                    Layout.fillWidth: true
                    Layout.topMargin: 8
                    
                    Text {
                        text: "Enable Real Sim"
                        font.pixelSize: 12
                        color: "#374151"
                        Layout.fillWidth: true
                    }
                    
                    Switch {
                        checked: simulationSetupPanel.enableRealSim
                        Layout.preferredWidth: 50
                        onCheckedChanged: simulationSetupPanel.enableRealSim = checked
                    }
                }
                
                // Divider
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 1
                    color: "#E5E7EB"
                    Layout.topMargin: 8
                    Layout.bottomMargin: 8
                }
                
                // Vehicle Message Fields section header
                Text {
                    text: "MESSAGE FIELDS"
                    font.pixelSize: 12
                    font.bold: true
                    color: "#9CA3AF"
                }
                
                // Radio button group
                ColumnLayout {
                    spacing: 8
                    Layout.fillWidth: true
                    
                    RowLayout {
                        spacing: 12
                        
                        RadioButton {
                            text: "Minimal Required"
                            font.pixelSize: 11
                            checked: simulationSetupPanel.vehicleMessageFieldsMode === "MinimalRequired"
                            onCheckedChanged: {
                                if (checked) simulationSetupPanel.vehicleMessageFieldsMode = "MinimalRequired"
                            }
                        }
                        
                        RadioButton {
                            text: "Common"
                            font.pixelSize: 11
                            checked: simulationSetupPanel.vehicleMessageFieldsMode === "Common"
                            onCheckedChanged: {
                                if (checked) simulationSetupPanel.vehicleMessageFieldsMode = "Common"
                            }
                        }
                    }
                    
                    RowLayout {
                        spacing: 12
                        
                        RadioButton {
                            text: "All"
                            font.pixelSize: 11
                            checked: simulationSetupPanel.vehicleMessageFieldsMode === "All"
                            onCheckedChanged: {
                                if (checked) simulationSetupPanel.vehicleMessageFieldsMode = "All"
                            }
                        }
                    }
                }
                
                // Selected fields display
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 44
                    color: "#F8FAFB"
                    radius: 6
                    border.color: "#E5E7EB"
                    border.width: 1
                    
                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 8
                        
                        Text {
                            text: simulationSetupPanel.selectedFields.join("  •  ")
                            font.pixelSize: 11
                            color: "#6B7280"
                            Layout.fillWidth: true
                        }
                        
                        Button {
                            text: "Customize >"
                            font.pixelSize: 10
                            Layout.preferredWidth: 90
                            Layout.preferredHeight: 28
                            flat: true
                            onClicked: {
                                messageFieldsPanel.selectedFields = simulationSetupPanel.selectedFields.slice()
                                messageFieldsPanel.open()
                            }
                        }
                    }
                }
                
                Item {
                    Layout.fillHeight: true
                }
            }
        }
        
        // Divider
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 1
            color: "#E5E7EB"
            Layout.bottomMargin: 16
            Layout.topMargin: 16
        }
        
        // Footer buttons
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 40
            spacing: 12
            
            Button {
                text: "Cancel"
                Layout.preferredWidth: 120
                Layout.preferredHeight: 40
                flat: true
                onClicked: {
                    setupCancelled()
                    simulationSetupPanel.close()
                }
            }
            
            Item {
                Layout.fillWidth: true
            }
            
            Button {
                text: "Save Configuration"
                Layout.preferredWidth: 150
                Layout.preferredHeight: 40
                font.bold: true
                highlighted: true
                onClicked: {
                    setupComplete()
                    simulationSetupPanel.close()
                }
            }
        }
    }
    
    // Select Vehicle Message Fields Panel
    SelectVehicleMessageFieldsPanel {
        id: messageFieldsPanel
        
        onFieldSelectionChanged: {
            simulationSetupPanel.selectedFields = messageFieldsPanel.selectedFields.slice()
        }
    }
}
