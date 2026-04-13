import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

/**
 * StatusRow.qml - A status checklist row with icon and label
 * Shows completion state with check or empty circle icon
 */
RowLayout {
    id: row
    
    property string label: "Status item"
    property bool isComplete: false
    
    spacing: 10
    Layout.fillWidth: true
    Layout.preferredHeight: 28
    
    // Status icon
    Rectangle {
        width: 20
        height: 20
        radius: 10
        border.color: row.isComplete ? "#3B82F6" : "#D1D5DB"
        border.width: 2
        color: row.isComplete ? "#3B82F6" : "transparent"
        Layout.alignment: Qt.AlignVCenter
        
        Text {
            visible: row.isComplete
            text: "✓"
            color: "white"
            font.bold: true
            font.pixelSize: 12
            anchors.centerIn: parent
        }
    }
    
    // Label
    Text {
        text: row.label
        font.pixelSize: 12
        color: "#4B5563"
        Layout.fillWidth: true
        Layout.alignment: Qt.AlignVCenter
    }
}
