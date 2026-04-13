import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

/**
 * XILBridgePanel.qml - Sidebar panel for configuring XIL Bridge
 * Opened when user clicks on the XIL Bridge config box
 */
Popup {
    id: xilBridgePanel
    width: 500
    height: parent ? parent.height * 0.85 : 600
    anchors.centerIn: parent
    padding: 0
    modal: true
    clip: true

    // Properties
    property bool xilBridgeEnabled: false
    property string xilBridgeMode: "Server"  // "Client" or "Server"
    property string selectedSimulator: "SUMO"
    property string carlaStatus: "Disabled"

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
                    text: "XIL Bridge"
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
                    onClicked: xilBridgePanel.close()
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
                            text: "Enable XIL Bridge"
                            font.pixelSize: 13
                            color: "#1F2937"
                            Layout.fillWidth: true
                        }

                        Switch {
                            id: enableSwitch
                            checked: xilBridgePanel.xilBridgeEnabled
                            Layout.preferredWidth: 50
                            onCheckedChanged: {
                                xilBridgePanel.xilBridgeEnabled = checked
                            }
                        }
                    }
                }

                // Mode Section
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    RowLayout {
                        spacing: 8
                        Layout.fillWidth: true

                        Text {
                            text: "Mode"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6B7280"
                            font.letterSpacing: 0.5
                        }

                        Item { Layout.fillWidth: true }
                    }

                    ColumnLayout {
                        spacing: 10
                        Layout.fillWidth: true

                        RadioButton {
                            text: "Client"
                            font.pixelSize: 13
                            checked: xilBridgePanel.xilBridgeMode === "Client"
                            onCheckedChanged: {
                                if (checked) {
                                    xilBridgePanel.xilBridgeMode = "Client"
                                }
                            }
                        }

                        RadioButton {
                            text: "Server"
                            font.pixelSize: 13
                            checked: xilBridgePanel.xilBridgeMode === "Server"
                            onCheckedChanged: {
                                if (checked) {
                                    xilBridgePanel.xilBridgeMode = "Server"
                                }
                            }
                        }
                    }
                }

                // Simulator Status Section
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 1
                        color: "#E5E7EB"
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
                            text: "Selected Simulator: " + xilBridgePanel.selectedSimulator
                            font.pixelSize: 12
                            color: "#1F2937"
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
                            color: "#EF4444"
                        }

                        Text {
                            text: "CARLA: " + xilBridgePanel.carlaStatus
                            font.pixelSize: 12
                            color: "#6B7280"
                            Layout.fillWidth: true
                        }
                    }
                }

                // Advanced Settings
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
                                text: "⚙"
                                font.pixelSize: 14
                            }

                            Text {
                                text: "Advanced Settings"
                                font.pixelSize: 13
                                font.bold: true
                                color: "#1F2937"
                                Layout.fillWidth: true
                            }

                            Text {
                                text: parent.expanded ? "▼" : "▶"
                                font.pixelSize: 12
                                color: "#9CA3AF"
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
                        height: parent.children[0].expanded ? 140 : 0
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

                            ColumnLayout {
                                spacing: 4
                                Layout.fillWidth: true

                                Text {
                                    text: "Server Address"
                                    font.pixelSize: 11
                                    color: "#6B7280"
                                }

                                TextField {
                                    text: "localhost"
                                    font.pixelSize: 11
                                    Layout.fillWidth: true
                                    Layout.preferredHeight: 32
                                }
                            }

                            ColumnLayout {
                                spacing: 4
                                Layout.fillWidth: true

                                Text {
                                    text: "Port"
                                    font.pixelSize: 11
                                    color: "#6B7280"
                                }

                                TextField {
                                    text: "2000"
                                    font.pixelSize: 11
                                    Layout.fillWidth: true
                                    Layout.preferredHeight: 32
                                }
                            }

                            CheckBox {
                                text: "Use Secure Connection"
                                font.pixelSize: 11
                                Layout.fillWidth: true
                            }
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

                    onClicked: xilBridgePanel.close()
                }

                Button {
                    text: "Save"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 38
                    Material.accent: "#3B82F6"

                    onClicked: {
                        console.log("XIL Bridge settings saved")
                        xilBridgePanel.close()
                    }
                }
            }
        }
    }
}
