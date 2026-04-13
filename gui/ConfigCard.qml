import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

/**
 * ConfigCard.qml - A progressive-reveal configuration card
 * 
 * Features:
 * - Smooth fade-in and slide-down animations
 * - Tracks completion state
 * - Can be programmatically shown/hidden
 * - Emits signals when completed or ready
 */
Rectangle {
    id: card
    
    // Properties
    property string title: "Section"
    property string subtitle: ""
    property bool cardVisible: false  // Controls visibility (triggers animation)
    property bool isComplete: false   // Tracks if step is complete
    property bool expanded: false
    
    // Signals
    signal completed()  // Emitted when user completes this step
    signal contentReady()  // Emitted when card is fully visible
    
    // Styling
    color: "#F0F4FF"
    border.color: "#E0E8FF"
    border.width: 2
    radius: 12
    
    // Layout
    Layout.fillWidth: true
    implicitHeight: expanded ? contentColumn.implicitHeight + 50 : 90
    
    // Initial state: invisible
    opacity: 0
    scale: 0.95
    
    Behavior on opacity {
        NumberAnimation {
            duration: 400
            easing.type: Easing.OutCubic
        }
    }
    
    Behavior on scale {
        NumberAnimation {
            duration: 400
            easing.type: Easing.OutCubic
        }
    }
    
    Behavior on implicitHeight {
        NumberAnimation {
            duration: 300
            easing.type: Easing.OutCubic
        }
    }
    
    // State machine: handle visibility changes
    onCardVisibleChanged: {
        if (cardVisible) {
            opacity = 1
            scale = 1
            contentReady()
        } else {
            opacity = 0
            scale = 0.95
        }
    }
    
    // Mark step complete with a signal
    function markComplete() {
        isComplete = true
        completed()
    }
    
    ColumnLayout {
        width: parent.width
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: 15
        spacing: 8
        
        // Header (clickable to expand)
        MouseArea {
            id: headerArea
            Layout.fillWidth: true
            Layout.preferredHeight: 60
            hoverEnabled: true
            onClicked: card.expanded = !card.expanded
            
            ColumnLayout {
                anchors.fill: parent
                anchors.rightMargin: 30
                spacing: 4
                
                Text {
                    text: card.title
                    font.pixelSize: 16
                    font.bold: true
                    color: "#1F2937"
                    Layout.fillWidth: true
                }
                
                Text {
                    text: card.subtitle
                    font.pixelSize: 12
                    color: "#6B7280"
                    Layout.fillWidth: true
                    visible: card.subtitle !== ""
                }
            }
            
            // Chevron indicator with smooth rotation
            Text {
                text: "▼"
                font.pixelSize: 20
                color: "#3B82F6"
                anchors.right: parent.right
                anchors.rightMargin: 5
                anchors.verticalCenter: parent.verticalCenter
                rotation: card.expanded ? 0 : -180
                
                Behavior on rotation {
                    NumberAnimation { duration: 200; easing.type: Easing.OutCubic }
                }
            }
            
            // Hover background
            Rectangle {
                anchors.fill: parent
                color: headerArea.containsMouse ? "#E0E8FF" : "transparent"
                radius: 6
                z: -1
                Behavior on color { ColorAnimation { duration: 200 } }
            }
        }
        
        // Expandable content area
        Item {
            Layout.fillWidth: true
            Layout.preferredHeight: card.expanded ? contentColumn.implicitHeight : 0
            clip: true
            visible: card.expanded
            
            Behavior on Layout.preferredHeight {
                NumberAnimation {
                    duration: 250
                    easing.type: Easing.OutCubic
                }
            }
            
            ColumnLayout {
                id: contentColumn
                width: parent.width
                spacing: 12
            }
        }
    }
    
    // Allow adding children to contentColumn
    default property alias content: contentColumn.children
}
