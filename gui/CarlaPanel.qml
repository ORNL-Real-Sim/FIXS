import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

/**
 * CarlaPanel.qml - Sidebar panel for configuring CARLA
 * Opened when user clicks on the CARLA config box
 */
Popup {
    id: carlaPanel
    width: 500
    height: parent ? parent.height * 0.85 : 600
    anchors.centerIn: parent
    padding: 0
    modal: true
    clip: true

    // Properties
    property bool carlaEnabled: false
    property string serverIp: "127.0.0.1"
    property string serverPort: "2000"
    property string selectedMap: "Town01"
    property string selectedSimulator: "SUMO"
    property bool applicationLayerEnabled: false
    property bool carlaConnected: false

    background: Rectangle {
        color: "#FFFFFF"
        radius: 0
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // Header
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 60
            color: "#F9FAFB"
            border.width: 1
            border.color: "#E5E7EB"

            RowLayout {
                anchors.fill: parent
                anchors.margins: 20
                spacing: 12

                Text {
                    text: "CARLA"
                    font.pixelSize: 18
                    font.bold: true
                    color: "#1F2937"
                    Layout.fillWidth: true
                }

                Button {
                    text: "✕"
                    Layout.preferredWidth: 36
                    Layout.preferredHeight: 36
                    font.pixelSize: 18
                    onClicked: carlaPanel.close()
                }
            }
        }

        // Divider
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 1
            color: "#E5E7EB"
        }

        // Content
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true

            ColumnLayout {
                width: parent.width
                spacing: 24
                anchors.margins: 24
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right

                // Enable Section
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    RowLayout {
                        spacing: 8
                        Layout.fillWidth: true

                        Text {
                            text: "Enable"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6B7280"
                            font.letterSpacing: 0.5
                        }

                        Item { Layout.fillWidth: true }
                    }

                    RowLayout {
                        spacing: 12
                        Layout.fillWidth: true

                        Text {
                            text: "Enable CARLA"
                            font.pixelSize: 13
                            color: "#1F2937"
                            Layout.fillWidth: true
                        }

                        Switch {
                            id: enableSwitch
                            checked: carlaPanel.carlaEnabled
                            Layout.preferredWidth: 50
                            onCheckedChanged: {
                                carlaPanel.carlaEnabled = checked
                            }
                        }
                    }
                }

                // Connection Section
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    RowLayout {
                        spacing: 8
                        Layout.fillWidth: true

                        Text {
                            text: "Connection"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6B7280"
                            font.letterSpacing: 0.5
                        }

                        Item { Layout.fillWidth: true }
                    }

                    ColumnLayout {
                        spacing: 8
                        Layout.fillWidth: true

                        // Server IP
                        ColumnLayout {
                            spacing: 4
                            Layout.fillWidth: true

                            Text {
                                text: "Server IP"
                                font.pixelSize: 11
                                color: "#6B7280"
                            }

                            TextField {
                                text: carlaPanel.serverIp
                                font.pixelSize: 12
                                Layout.fillWidth: true
                                Layout.preferredHeight: 36
                                onEditingFinished: carlaPanel.serverIp = text
                            }
                        }

                        // Server Port
                        ColumnLayout {
                            spacing: 4
                            Layout.fillWidth: true

                            Text {
                                text: "Server Port"
                                font.pixelSize: 11
                                color: "#6B7280"
                            }

                            TextField {
                                text: carlaPanel.serverPort
                                font.pixelSize: 12
                                Layout.fillWidth: true
                                Layout.preferredHeight: 36
                                onEditingFinished: carlaPanel.serverPort = text
                            }
                        }
                    }
                }

                // Environment Section
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    RowLayout {
                        spacing: 8
                        Layout.fillWidth: true

                        Text {
                            text: "Environment"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6B7280"
                            font.letterSpacing: 0.5
                        }

                        Item { Layout.fillWidth: true }
                    }

                    ColumnLayout {
                        spacing: 4
                        Layout.fillWidth: true

                        Text {
                            text: "Map"
                            font.pixelSize: 11
                            color: "#6B7280"
                        }

                        ComboBox {
                            model: ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD"]
                            currentIndex: 0
                            Layout.fillWidth: true
                            Layout.preferredHeight: 36
                        }
                    }
                }

                // Optional Section
                ColumnLayout {
                    spacing: 8
                    Layout.fillWidth: true

                    MouseArea {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        hoverEnabled: true

                        property bool expanded: false

                        onClicked: expanded = !expanded

                        RowLayout {
                            anchors.fill: parent
                            anchors.rightMargin: 12
                            spacing: 12

                            Text {
                                text: "▶"
                                font.pixelSize: 12
                                color: "#9CA3AF"
                            }

                            Text {
                                text: "Optional"
                                font.pixelSize: 13
                                font.bold: true
                                color: "#1F2937"
                                Layout.fillWidth: true
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

                    Item {
                        Layout.fillWidth: true
                        height: parent.children[0].expanded ? 100 : 0
                        clip: true
                        visible: height > 0

                        Behavior on height {
                            NumberAnimation { duration: 250; easing.type: Easing.OutCubic }
                        }

                        ColumnLayout {
                            width: parent.width
                            anchors.top: parent.top
                            anchors.margins: 8
                            spacing: 12

                            CheckBox {
                                text: "Render visualization"
                                font.pixelSize: 11
                                Layout.fillWidth: true
                            }

                            CheckBox {
                                text: "Enable traffic manager"
                                font.pixelSize: 11
                                Layout.fillWidth: true
                            }

                            CheckBox {
                                text: "Synchronous mode"
                                font.pixelSize: 11
                                Layout.fillWidth: true
                            }
                        }
                    }
                }

                // Status Section
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 1
                        color: "#E5E7EB"
                    }

                    Text {
                        text: "Status"
                        font.pixelSize: 12
                        font.bold: true
                        color: "#6B7280"
                        font.letterSpacing: 0.5
                    }

                    // Selected Simulator
                    RowLayout {
                        spacing: 12
                        Layout.fillWidth: true

                        Text {
                            text: "⚙"
                            font.pixelSize: 14
                            color: "#6B7280"
                        }

                        Text {
                            text: "Selected Simulator: " + carlaPanel.selectedSimulator
                            font.pixelSize: 12
                            color: "#1F2937"
                            Layout.fillWidth: true
                        }
                    }

                    // Application Layer Status
                    RowLayout {
                        spacing: 12
                        Layout.fillWidth: true

                        Text {
                            text: "●"
                            font.pixelSize: 14
                            color: "#E5E7EB"
                        }

                        Text {
                            text: "Application Layer: Disabled"
                            font.pixelSize: 12
                            color: "#6B7280"
                            Layout.fillWidth: true
                        }
                    }

                    // CARLA Status
                    RowLayout {
                        spacing: 12
                        Layout.fillWidth: true

                        Text {
                            text: "●"
                            font.pixelSize: 14
                            color: carlaPanel.carlaConnected ? "#10B981" : "#10B981"
                        }

                        Text {
                            text: "CARLA: " + (carlaPanel.carlaEnabled ? "Enabled" : "Disabled")
                            font.pixelSize: 12
                            color: "#1F2937"
                            Layout.fillWidth: true
                        }
                    }
                }

                Item { Layout.fillHeight: true }
            }
        }

        // Footer buttons
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 70
            color: "#F9FAFB"
            border.width: 1
            border.color: "#E5E7EB"

            RowLayout {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 12

                Button {
                    text: "Cancel"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 38

                    onClicked: carlaPanel.close()
                }

                Button {
                    text: "Save"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 38
                    Material.accent: "#3B82F6"

                    onClicked: {
                        console.log("CARLA settings saved")
                        carlaPanel.close()
                    }
                }
            }
        }
    }
}
