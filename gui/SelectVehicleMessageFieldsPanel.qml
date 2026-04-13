import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

/**
 * SelectVehicleMessageFieldsPanel.qml - Modal for selecting vehicle message fields
 * Provides searchable checkboxes, preset buttons, and tag display
 */
Popup {
    id: messageFieldsPanel
    modal: true
    focus: true
    closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
    
    // Signals
    signal fieldSelectionChanged()
    
    // Properties
    property var selectedFields: ["id", "type", "vehicleClass", "speed", "speedDesired", "accelerationDesiredSmoothed"]
    property string searchText: ""
    
    width: 650
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
                text: "Select Vehicle Message Fields"
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
                onClicked: messageFieldsPanel.close()
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
        
        // Search bar
        TextField {
            id: searchField
            Layout.fillWidth: true
            Layout.preferredHeight: 40
            placeholderText: "Search..."
            font.pixelSize: 12
            onTextChanged: messageFieldsPanel.searchText = text
            
            background: Rectangle {
                color: "#F3F4F6"
                radius: 6
                border.color: searchField.focus ? "#3B82F6" : "#E5E7EB"
                border.width: 2
            }
        }
        
        Item {
            Layout.preferredHeight: 8
        }
        
        // Preset buttons
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            
            Button {
                text: "Minimal"
                font.pixelSize: 11
                Layout.preferredHeight: 32
                flat: true
                onClicked: {
                    messageFieldsPanel.selectedFields = ["id", "speed", "speedDesired"]
                    messageFieldsPanel.fieldSelectionChanged()
                }
            }
            
            Button {
                text: "All"
                font.pixelSize: 11
                Layout.preferredHeight: 32
                flat: true
                onClicked: {
                    messageFieldsPanel.selectedFields = allFieldsList.slice()
                    messageFieldsPanel.fieldSelectionChanged()
                }
            }
            
            Button {
                text: "SUMO Minimal"
                font.pixelSize: 11
                Layout.preferredHeight: 32
                flat: true
                onClicked: {
                    messageFieldsPanel.selectedFields = ["id", "type", "speed", "speedDesired", "acceleration"]
                    messageFieldsPanel.fieldSelectionChanged()
                }
            }
            
            Item {
                Layout.fillWidth: true
            }
        }
        
        Item {
            Layout.preferredHeight: 12
        }
        
        // Checkboxes grid
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            GridLayout {
                width: messageFieldsPanel.width - 48
                columns: 3
                columnSpacing: 16
                rowSpacing: 12
                
                // All available fields
                CheckBox {
                    text: "posX"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("posX")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("posX")) {
                                messageFieldsPanel.selectedFields.push("posX")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "posX")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "posY"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("posY")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("posY")) {
                                messageFieldsPanel.selectedFields.push("posY")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "posY")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "angle"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("angle")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("angle")) {
                                messageFieldsPanel.selectedFields.push("angle")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "angle")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "id"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("id")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("id")) {
                                messageFieldsPanel.selectedFields.push("id")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "id")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "type"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("type")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("type")) {
                                messageFieldsPanel.selectedFields.push("type")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "type")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "vehicleClass"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("vehicleClass")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("vehicleClass")) {
                                messageFieldsPanel.selectedFields.push("vehicleClass")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "vehicleClass")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "laneIndex"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("laneIndex")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("laneIndex")) {
                                messageFieldsPanel.selectedFields.push("laneIndex")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "laneIndex")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "accelerationSmoothed"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("accelerationSmoothed")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("accelerationSmoothed")) {
                                messageFieldsPanel.selectedFields.push("accelerationSmoothed")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "accelerationSmoothed")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "accelerationDesired"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("accelerationDesired")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("accelerationDesired")) {
                                messageFieldsPanel.selectedFields.push("accelerationDesired")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "accelerationDesired")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "acceleration"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("acceleration")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("acceleration")) {
                                messageFieldsPanel.selectedFields.push("acceleration")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "acceleration")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "speed"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("speed")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("speed")) {
                                messageFieldsPanel.selectedFields.push("speed")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "speed")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "posZ"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("posZ")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("posZ")) {
                                messageFieldsPanel.selectedFields.push("posZ")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "posZ")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "yaw"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("yaw")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("yaw")) {
                                messageFieldsPanel.selectedFields.push("yaw")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "yaw")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "speedDesiredSmoothed"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("speedDesiredSmoothed")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("speedDesiredSmoothed")) {
                                messageFieldsPanel.selectedFields.push("speedDesiredSmoothed")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "speedDesiredSmoothed")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "mass"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("mass")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("mass")) {
                                messageFieldsPanel.selectedFields.push("mass")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "mass")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "radius"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("radius")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("radius")) {
                                messageFieldsPanel.selectedFields.push("radius")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "radius")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "fuelConsumed"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("fuelConsumed")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("fuelConsumed")) {
                                messageFieldsPanel.selectedFields.push("fuelConsumed")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "fuelConsumed")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "stateUserData"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("stateUserData")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("stateUserData")) {
                                messageFieldsPanel.selectedFields.push("stateUserData")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "stateUserData")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "speedDesired"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("speedDesired")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("speedDesired")) {
                                messageFieldsPanel.selectedFields.push("speedDesired")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "speedDesired")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
                
                CheckBox {
                    text: "accelerationDesiredSmoothed"
                    font.pixelSize: 11
                    checked: messageFieldsPanel.selectedFields.includes("accelerationDesiredSmoothed")
                    onCheckedChanged: {
                        if (checked) {
                            if (!messageFieldsPanel.selectedFields.includes("accelerationDesiredSmoothed")) {
                                messageFieldsPanel.selectedFields.push("accelerationDesiredSmoothed")
                            }
                        } else {
                            messageFieldsPanel.selectedFields = messageFieldsPanel.selectedFields.filter(f => f !== "accelerationDesiredSmoothed")
                        }
                        messageFieldsPanel.fieldSelectionChanged()
                    }
                }
            }
        }
        
        Item {
            Layout.preferredHeight: 12
        }
        
        // Divider
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 1
            color: "#E5E7EB"
        }
        
        Item {
            Layout.preferredHeight: 12
        }
        
        // Selected fields tags
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            
            Flickable {
                Layout.fillWidth: true
                Layout.preferredHeight: 32
                contentWidth: tagsRow.width
                contentHeight: tagsRow.height
                clip: true
                flickableDirection: Flickable.HorizontalFlick
                
                RowLayout {
                    id: tagsRow
                    spacing: 8
                    
                    Repeater {
                        model: messageFieldsPanel.selectedFields
                        
                        Rectangle {
                            Layout.preferredHeight: 28
                            Layout.preferredWidth: tags.width + 12
                            color: "#E8ECFF"
                            radius: 14
                            border.color: "#D4DCFF"
                            border.width: 1
                            
                            Text {
                                id: tags
                                text: modelData
                                font.pixelSize: 10
                                color: "#6B7280"
                                anchors.centerIn: parent
                                anchors.margins: 6
                            }
                        }
                    }
                }
            }
            
            Button {
                text: ">"
                font.pixelSize: 12
                Layout.preferredWidth: 32
                Layout.preferredHeight: 32
                flat: true
            }
        }
        
        Item {
            Layout.preferredHeight: 12
        }
        
        // Footer buttons
        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            
            Item {
                Layout.fillWidth: true
            }
            
            Button {
                text: "Close"
                Layout.preferredWidth: 100
                Layout.preferredHeight: 40
                flat: true
                onClicked: messageFieldsPanel.close()
            }
        }
    }
    
    // List of all available fields
    property var allFieldsList: [
        "posX", "posY", "angle", "id", "type", "vehicleClass",
        "laneIndex", "accelerationSmoothed", "accelerationDesired",
        "acceleration", "speed", "posZ", "yaw", "speedDesiredSmoothed",
        "mass", "radius", "fuelConsumed", "stateUserData", "speedDesired",
        "accelerationDesiredSmoothed"
    ]
}
