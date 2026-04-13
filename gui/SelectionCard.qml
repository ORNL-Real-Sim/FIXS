import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

/**
 * SelectionCard.qml - Premium centered selection card
 * Used for presenting large interactive options in the main content area
 */
Rectangle {
    id: card
    
    property string title: "Title"
    property string subtitle: "Subtitle"
    signal selected()
    
    color: "#F0F4FF"
    border.color: "#D4DCFF"
    border.width: 1
    radius: 16
    
    Layout.preferredHeight: 220
    Layout.preferredWidth: 400
    
    // Hover effect
    property bool isHovered: mouseArea.containsMouse
    
    Behavior on color {
        ColorAnimation { duration: 150 }
    }
    
    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true
        onClicked: card.selected()
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 40
            spacing: 12
            
            Text {
                text: card.title
                font.pixelSize: 22
                font.bold: true
                color: "#1F2937"
                Layout.alignment: Qt.AlignHCenter
            }
            
            Text {
                text: card.subtitle
                font.pixelSize: 14
                color: "#6B7280"
                Layout.alignment: Qt.AlignHCenter
            }
        }
    }
    
    // Subtle hover background shift
    Rectangle {
        anchors.fill: parent
        radius: 16
        color: isHovered ? "#E8ECFF" : "transparent"
        z: -1
        Behavior on color { ColorAnimation { duration: 150 } }
    }
}
