import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

/**
 * ApplicationLayerPanel.qml - Sidebar panel for configuring Application Layer
 * Opened when user clicks on the Application Layer config box
 */
Popup {
    id: appLayerPanel
    width: 500
    height: parent ? parent.height * 0.85 : 600
    anchors.centerIn: parent
    padding: 0
    modal: true
    clip: true

    // Properties
    property bool applicationLayerEnabled: false
    property var selectedDataFields: ["Vehicles", "Signals IDs"]
    property var subscriptions: [
        { id: 1, name: "Vehicle Subscription: Egos", filter: "All vehicles", output: "IP: 127.0.0.1, Port: 8000" }
    ]

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
                    text: "Application Layer"
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
                    onClicked: appLayerPanel.close()
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

                // Step 1: Enable
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    RowLayout {
                        spacing: 8
                        Layout.fillWidth: true

                        Text {
                            text: "Step 1: Enable"
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
                            text: "Enable Application Layer"
                            font.pixelSize: 13
                            color: "#1F2937"
                            Layout.fillWidth: true
                        }

                        Switch {
                            id: enableSwitch
                            checked: appLayerPanel.applicationLayerEnabled
                            Layout.preferredWidth: 50
                            onCheckedChanged: {
                                appLayerPanel.applicationLayerEnabled = checked
                            }
                        }
                    }
                }

                // Step 2: Select data fields
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    RowLayout {
                        spacing: 8
                        Layout.fillWidth: true

                        Text {
                            text: "Step 2: What data would you like to receive?"
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

                        Repeater {
                            model: ["Vehicles", "Signals IDs", "Radius around point"]

                            CheckBox {
                                text: modelData
                                font.pixelSize: 13
                                Layout.fillWidth: true
                                checked: appLayerPanel.selectedDataFields.includes(modelData)
                                onCheckedChanged: {
                                    if (checked && !appLayerPanel.selectedDataFields.includes(modelData)) {
                                        appLayerPanel.selectedDataFields.push(modelData)
                                    } else if (!checked) {
                                        appLayerPanel.selectedDataFields = appLayerPanel.selectedDataFields.filter(
                                            item => item !== modelData
                                        )
                                    }
                                }
                            }
                        }
                    }
                }

                // Step 3: Build subscriptions
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    RowLayout {
                        spacing: 8
                        Layout.fillWidth: true

                        Text {
                            text: "Step 3: Build your subscriptions"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6B7280"
                            font.letterSpacing: 0.5
                        }

                        Item { Layout.fillWidth: true }
                    }

                    Button {
                        text: "+ Add Subscription"
                        Layout.fillWidth: true
                        Layout.preferredHeight: 36
                        Material.accent: "#3B82F6"
                        font.pixelSize: 12
                        font.bold: true

                        onClicked: {
                            appLayerPanel.subscriptions.push({
                                id: appLayerPanel.subscriptions.length + 1,
                                name: "New Subscription",
                                filter: "All vehicles",
                                output: "IP: 127.0.0.1, Port: 8000"
                            })
                            subscriptionsRepeater.model = appLayerPanel.subscriptions
                        }
                    }
                }

                // Subscriptions list
                ColumnLayout {
                    spacing: 12
                    Layout.fillWidth: true

                    Repeater {
                        id: subscriptionsRepeater
                        model: appLayerPanel.subscriptions

                        ColumnLayout {
                            spacing: 12
                            Layout.fillWidth: true

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 1
                                color: "#E5E7EB"
                            }

                            // Subscription header
                            RowLayout {
                                spacing: 12
                                Layout.fillWidth: true

                                Text {
                                    text: modelData.name
                                    font.pixelSize: 13
                                    font.bold: true
                                    color: "#1F2937"
                                    Layout.fillWidth: true
                                }

                                Button {
                                    text: "✏"
                                    Layout.preferredWidth: 32
                                    Layout.preferredHeight: 32
                                    font.pixelSize: 14
                                }
                            }

                            // Filter and Output
                            ColumnLayout {
                                spacing: 12
                                Layout.fillWidth: true

                                RowLayout {
                                    spacing: 12
                                    Layout.fillWidth: true

                                    ColumnLayout {
                                        spacing: 4
                                        Layout.fillWidth: true

                                        Text {
                                            text: "Filter"
                                            font.pixelSize: 11
                                            color: "#6B7280"
                                        }

                                        RowLayout {
                                            spacing: 6
                                            Layout.fillWidth: true

                                            RadioButton {
                                                text: "All vehicles"
                                                font.pixelSize: 11
                                                checked: modelData.filter === "All vehicles"
                                            }

                                            RadioButton {
                                                text: "Specific IDs"
                                                font.pixelSize: 11
                                            }

                                            RadioButton {
                                                text: "Radius around point"
                                                font.pixelSize: 11
                                            }
                                        }
                                    }

                                    ColumnLayout {
                                        spacing: 4
                                        Layout.preferredWidth: 200

                                        Text {
                                            text: "Output"
                                            font.pixelSize: 11
                                            color: "#6B7280"
                                        }

                                        RowLayout {
                                            spacing: 8
                                            Layout.fillWidth: true

                                            ColumnLayout {
                                                spacing: 2
                                                Layout.preferredWidth: 80

                                                Text {
                                                    text: "IP"
                                                    font.pixelSize: 10
                                                    color: "#6B7280"
                                                }

                                                TextField {
                                                    text: "127.0.0.1"
                                                    font.pixelSize: 11
                                                    Layout.fillWidth: true
                                                    Layout.preferredHeight: 28
                                                }
                                            }

                                            ColumnLayout {
                                                spacing: 2
                                                Layout.preferredWidth: 70

                                                Text {
                                                    text: "Port"
                                                    font.pixelSize: 10
                                                    color: "#6B7280"
                                                }

                                                TextField {
                                                    text: "8000"
                                                    font.pixelSize: 11
                                                    Layout.fillWidth: true
                                                    Layout.preferredHeight: 28
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            // Coordinates for radius (if radius is selected)
                            RowLayout {
                                spacing: 8
                                Layout.fillWidth: true

                                Text {
                                    text: "X"
                                    font.pixelSize: 10
                                    color: "#6B7280"
                                }

                                TextField {
                                    placeholderText: "X"
                                    font.pixelSize: 11
                                    Layout.preferredWidth: 40
                                    Layout.preferredHeight: 28
                                }

                                Text {
                                    text: "Y"
                                    font.pixelSize: 10
                                    color: "#6B7280"
                                }

                                TextField {
                                    placeholderText: "Y"
                                    font.pixelSize: 11
                                    Layout.preferredWidth: 40
                                    Layout.preferredHeight: 28
                                }

                                Text {
                                    text: "Radius"
                                    font.pixelSize: 10
                                    color: "#6B7280"
                                }

                                TextField {
                                    placeholderText: "Radius"
                                    font.pixelSize: 11
                                    Layout.preferredWidth: 60
                                    Layout.preferredHeight: 28
                                }

                                Item { Layout.fillWidth: true }
                            }
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
                                text: "👁"
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
                        height: parent.children[0].expanded ? 80 : 0
                        clip: true
                        visible: height > 0

                        Behavior on height {
                            NumberAnimation { duration: 250; easing.type: Easing.OutCubic }
                        }

                        ColumnLayout {
                            width: parent.width
                            anchors.top: parent.top
                            anchors.margins: 8
                            spacing: 8

                            CheckBox {
                                text: "Enable verbose logging"
                                font.pixelSize: 11
                                Layout.fillWidth: true
                            }

                            CheckBox {
                                text: "Include timestamp"
                                font.pixelSize: 11
                                Layout.fillWidth: true
                            }
                        }
                    }
                }

                // Preview YAML
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
                                text: "Preview YAML"
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
                        height: parent.children[0].expanded ? 150 : 0
                        clip: true
                        visible: height > 0

                        Behavior on height {
                            NumberAnimation { duration: 250; easing.type: Easing.OutCubic }
                        }

                        Rectangle {
                            width: parent.width
                            height: parent.height - 8
                            anchors.top: parent.top
                            anchors.margins: 8
                            color: "#1F2937"
                            radius: 6
                            border.color: "#374151"
                            border.width: 1

                            TextEdit {
                                anchors.fill: parent
                                anchors.margins: 12
                                color: "#10B981"
                                font.family: "Courier"
                                font.pixelSize: 11
                                readOnly: true
                                text: "ApplicationLayer:\n  enabled: true\n  dataFields:\n    - Vehicles\n    - Signals IDs\n  subscriptions: [...]"
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

                    onClicked: appLayerPanel.close()
                }

                Button {
                    text: "Save"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 38
                    Material.accent: "#3B82F6"

                    onClicked: {
                        // Prepare the Application Layer data as JSON
                        var appLayerData = {
                            applicationLayerEnabled: appLayerPanel.applicationLayerEnabled,
                            selectedDataFields: appLayerPanel.selectedDataFields,
                            subscriptions: appLayerPanel.subscriptions
                        };
                        
                        // Call the backend to save and download the file
                        configManager.saveApplicationLayerConfig(JSON.stringify(appLayerData));
                        appLayerPanel.close()
                    }
                }
            }
        }
    }
}
