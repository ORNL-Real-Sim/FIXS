import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtQuick.Effects
import "."

ApplicationWindow {
    id: root
    visible: true
    width: 1280
    height: 720
    title: "FIXS Configuration"
    color: "#F8FAFB"

    // State management
    property string currentState: "start"  // States: "start", "sidebar_open", "main_config"
    
    // Configuration data
    property string selectedSimulator: ""
    property real simulationDuration: 60
    property bool applicationLayerEnabled: false
    property bool carmakerEnabled: false
    property bool advancedSettingsExpanded: false

    // Simulator options
    property var simulatorOptions: [
        { name: "SUMO", display: "SUMO", port: 8813 },
        { name: "CARLA", display: "CARLA", port: 2000 },
        { name: "VISSIM", display: "VISSIM", port: 6789 },
        { name: "IPG_CarMaker", display: "IPG CarMaker", port: 8012 }
    ]

    // Configuration sections (displayed in main_config state)
    property var configSections: [
        { id: 1, title: "Simulation Setup", subtitle: "Configure simulation", color: "#E8ECFF" },
        { id: 2, title: "Application Layer", subtitle: "Set message fields", color: "#E8ECFF" },
        { id: 3, title: "XIL Bridge", subtitle: "Simulink/CarMaker integration", color: "#E8ECFF" },
        { id: 4, title: "CARLA", subtitle: "Co-simulation", color: "#E8ECFF" }
    ]

    // ============================================================================
    // STEP 1: START STATE (Single simulator card)
    // ============================================================================
    Rectangle {
        id: startStateView
        anchors.fill: parent
        color: "#F8FAFB"
        opacity: root.currentState === "start" ? 1 : 0
        visible: opacity > 0

        Behavior on opacity {
            NumberAnimation { duration: 300; easing.type: Easing.InOutQuad }
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 40
            spacing: 30

            Item { Layout.fillHeight: true; Layout.minimumHeight: 20 }

            // Title
            ColumnLayout {
                spacing: 8
                Layout.alignment: Qt.AlignHCenter

                Text {
                    text: "FIXS Configuration"
                    font.pixelSize: 40
                    font.weight: Font.Bold
                    color: "#1F2937"
                    Layout.alignment: Qt.AlignHCenter
                }

                Text {
                    text: "Select your traffic simulator to get started"
                    font.pixelSize: 16
                    color: "#6B7280"
                    Layout.alignment: Qt.AlignHCenter
                }
            }

            Item { Layout.fillHeight: true }

            // Single Traffic Simulator Card
            SelectionCard {
                id: startCard
                title: "Traffic Simulator"
                subtitle: "Choose: SUMO, CARLA, VISSIM, or IPG CarMaker"
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 500
                Layout.preferredHeight: 240

                onSelected: {
                    console.log("Start card clicked - transitioning to sidebar_open")
                    root.currentState = "sidebar_open"
                }
            }

            Item { Layout.fillHeight: true }
        }
    }

    // ============================================================================
    // STEP 2: SIDEBAR OPEN STATE (Simulator card + sidebar with config)
    // ============================================================================
    Rectangle {
        id: sidebarStateView
        anchors.fill: parent
        color: "#F8FAFB"
        opacity: root.currentState === "sidebar_open" ? 1 : 0
        visible: opacity > 0

        Behavior on opacity {
            NumberAnimation { duration: 300; easing.type: Easing.InOutQuad }
        }

        RowLayout {
            anchors.fill: parent
            spacing: 0

            // Left: Simulator Card
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#FAFBFC"

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 40
                    anchors.topMargin: 80
                    spacing: 20

                    Text {
                        text: "Step 1: Select Simulator"
                        font.pixelSize: 28
                        font.weight: Font.Bold
                        color: "#1F2937"
                        Layout.alignment: Qt.AlignHCenter
                    }

                    SelectionCard {
                        id: sidebarCard
                        title: "Traffic Simulator"
                        subtitle: "Choose your simulator"
                        Layout.preferredWidth: 400
                        Layout.preferredHeight: 200
                        Layout.alignment: Qt.AlignHCenter

                        onSelected: {
                            // When card is clicked in this state, show simulator options
                            simulatorSelectionSheet.visible = true
                        }
                    }
                }
            }

            // Vertical Divider
            Rectangle {
                Layout.fillHeight: true
                Layout.preferredWidth: 1
                color: "#E5E7EB"
            }

            // Right: Sidebar
            SetupSidebar {
                id: sidebarPanel
                Layout.preferredWidth: 340
                Layout.fillHeight: true

                stepLabel: "Configure Simulator"
                selectedSimulator: root.selectedSimulator
                simulationDuration: root.simulationDuration
                applicationLayerEnabled: root.applicationLayerEnabled
                carmakerEnabled: root.carmakerEnabled
                advancedExpanded: root.advancedSettingsExpanded

                onDoneClicked: {
                    console.log("Sidebar Done clicked - transitioning to main_config")
                    root.currentState = "main_config"
                }

                onSimulatorSelected: {
                    root.selectedSimulator = name
                }
            }
        }
    }

    // ============================================================================
    // STEP 3: MAIN CONFIG STATE (4 configuration boxes)
    // ============================================================================
    Rectangle {
        id: mainConfigView
        anchors.fill: parent
        color: "#F8FAFB"
        opacity: root.currentState === "main_config" ? 1 : 0
        visible: opacity > 0

        Behavior on opacity {
            NumberAnimation { duration: 300; easing.type: Easing.InOutQuad }
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 40
            spacing: 20

            // Header
            ColumnLayout {
                spacing: 8
                Layout.fillWidth: true

                Text {
                    text: "FIXS Configuration"
                    font.pixelSize: 32
                    font.weight: Font.Bold
                    color: "#1F2937"
                }

                Text {
                    text: "Simulator: " + root.selectedSimulator + " | Duration: " + root.simulationDuration + "s"
                    font.pixelSize: 13
                    color: "#6B7280"
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 1
                    color: "#E5E7EB"
                }
            }

            // Config sections grid
            GridLayout {
                columns: 2
                rowSpacing: 20
                columnSpacing: 20
                Layout.fillWidth: true
                Layout.fillHeight: true

                Repeater {
                    model: root.configSections

                    ConfigBox {
                        title: modelData.title
                        subtitle: modelData.subtitle
                        backgroundColor: modelData.color
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.preferredHeight: 250

                        // Fade-in animation
                        opacity: 0
                        Behavior on opacity {
                            NumberAnimation {
                                from: 0
                                to: 1
                                duration: 500
                                easing.type: Easing.OutCubic
                            }
                        }

                        Component.onCompleted: {
                            opacity = 1
                        }

                        onBoxClicked: function(title) {
                            if (title === "Simulation Setup") {
                                appSetupPanel.open()
                            } else if (title === "Application Layer") {
                                appLayerPanel.open()
                            } else if (title === "XIL Bridge") {
                                xilBridgePanel.open()
                            } else if (title === "CARLA") {
                                carlaPanel.open()
                            }
                        }
                    }
                }
            }

            // Action buttons at bottom
            RowLayout {
                Layout.fillWidth: true
                spacing: 12

                Button {
                    text: "← Back"
                    Layout.preferredWidth: 120
                    font.pixelSize: 12

                    onClicked: {
                        root.currentState = "sidebar_open"
                    }
                }

                Item { Layout.fillWidth: true }

                Button {
                    text: "Save Configuration"
                    Layout.preferredWidth: 180
                    font.pixelSize: 12
                    Material.accent: "#3B82F6"

                    onClicked: {
                        configManager.saveConfig()
                        console.log("Configuration saved - opening Application Setup")
                        appSetupPanel.open()
                    }
                }
            }
        }
    }

    // ============================================================================
    // Simulator Selection Sheet (appears when card clicked in sidebar state)
    // ============================================================================
    Popup {
        id: simulatorSelectionSheet
        anchors.centerIn: parent
        width: 500
        height: 450
        padding: 0
        visible: false

        background: Rectangle {
            color: "#FFFFFF"
            radius: 12
            border.color: "#E5E7EB"
            border.width: 1
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 24
            spacing: 16

            Text {
                text: "Select Traffic Simulator"
                font.pixelSize: 18
                font.bold: true
                color: "#1F2937"
                Layout.fillWidth: true
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 1
                color: "#E5E7EB"
            }

            ScrollView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true

                ColumnLayout {
                    width: parent.width
                    spacing: 10

                    Repeater {
                        model: root.simulatorOptions

                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 60
                            radius: 8
                            color: root.selectedSimulator === modelData.name ? "#EFF6FF" : "#F9FAFB"
                            border.color: root.selectedSimulator === modelData.name ? "#3B82F6" : "#E5E7EB"
                            border.width: 2

                            MouseArea {
                                anchors.fill: parent
                                hoverEnabled: true

                                onClicked: {
                                    root.selectedSimulator = modelData.name
                                }

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 12
                                    spacing: 2

                                    Text {
                                        text: modelData.display
                                        font.pixelSize: 14
                                        font.bold: true
                                        color: "#1F2937"
                                    }

                                    Text {
                                        text: "Port: " + modelData.port
                                        font.pixelSize: 11
                                        color: "#6B7280"
                                    }
                                }
                            }

                            Behavior on color {
                                ColorAnimation { duration: 150 }
                            }
                        }
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 1
                color: "#E5E7EB"
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 12

                Button {
                    text: "Cancel"
                    Layout.fillWidth: true

                    onClicked: {
                        simulatorSelectionSheet.close()
                    }
                }

                Button {
                    text: "Select"
                    Layout.fillWidth: true
                    Material.accent: "#3B82F6"

                    onClicked: {
                        simulatorSelectionSheet.close()
                    }
                }
            }
        }
    }

    // ============================================================================
    // Simulation Setup Configuration Panel
    // ============================================================================
    SimulationSetupPanel {
        id: appSetupPanel
        
        onSetupComplete: {
            appLayerPanel.open()
        }
    }

    // ============================================================================
    // Application Layer Configuration Panel
    // ============================================================================
    ApplicationLayerPanel {
        id: appLayerPanel
    }

    // ============================================================================
    // XIL Bridge Configuration Panel
    // ============================================================================
    XILBridgePanel {
        id: xilBridgePanel
        selectedSimulator: root.selectedSimulator
    }

    // ============================================================================
    // CARLA Configuration Panel
    // ============================================================================
    CarlaPanel {
        id: carlaPanel
        selectedSimulator: root.selectedSimulator
    }

    // Load config on startup
    Component.onCompleted: {
        configManager.loadConfig()
    }
}
