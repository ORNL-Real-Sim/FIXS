import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

/**
 * ConfigBox.qml - Collapsible configuration section box
 * Used in the main configuration view to display different setup areas
 */
Rectangle {
    id: configBox
    
    // Properties
    property string title: "Configuration Section"
    property string subtitle: "Add your settings here"
    property string backgroundColor: "#E8ECFF"
    property bool isExpanded: false
    
    // Signals
    signal boxClicked(string title)
    
    radius: 12
    color: backgroundColor
    border.color: "#D4DCFF"
    border.width: 1
    clip: true

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 16

        // Header (clickable to expand/collapse)
        MouseArea {
            id: headerArea
            Layout.fillWidth: true
            Layout.preferredHeight: 50
            hoverEnabled: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 4

                Text {
                    text: configBox.title
                    font.pixelSize: 16
                    font.bold: true
                    font.weight: Font.Bold
                    color: "#1F2937"
                    Layout.fillWidth: true
                }

                Text {
                    text: configBox.subtitle
                    font.pixelSize: 12
                    color: "#6B7280"
                    Layout.fillWidth: true
                }
            }

            onClicked: {
                configBox.isExpanded = !configBox.isExpanded
                configBox.boxClicked(configBox.title)
            }

            // Hover background
            Rectangle {
                anchors.fill: parent
                color: headerArea.containsMouse ? "rgba(0, 0, 0, 0.02)" : "transparent"
                radius: 8
                z: -1
                Behavior on color { ColorAnimation { duration: 150 } }
            }
        }

        // Divider
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 1
            color: "#D4DCFF"
        }

        // Expandable content area
        Item {
            Layout.fillWidth: true
            height: configBox.isExpanded ? 120 : 0
            clip: true
            visible: height > 0

            Behavior on height {
                NumberAnimation { duration: 300; easing.type: Easing.OutCubic }
            }

            ColumnLayout {
                width: parent.width
                anchors.top: parent.top
                anchors.margins: 12
                spacing: 12

                Text {
                    text: "Configuration options will appear here"
                    font.pixelSize: 12
                    color: "#6B7280"
                    Layout.fillWidth: true
                    wrapMode: Text.WordWrap
                }

                Row {
                    spacing: 8
                    Layout.fillWidth: true

                    Button {
                        text: "Edit"
                        font.pixelSize: 11
                    }

                    Button {
                        text: "Reset"
                        font.pixelSize: 11
                    }
                }
            }
        }

        Item { Layout.fillHeight: true }
    }
}
