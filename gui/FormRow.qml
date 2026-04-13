import QtQuick
import QtQuick.Layouts

RowLayout {
    property string label: "Label"
    spacing: 12
    Layout.fillWidth: true

    Text {
        text: label
        font.pixelSize: 12
        color: "#1F2937"
        Layout.preferredWidth: 180
    }

    Item {
        Layout.fillWidth: true
    }

    default property alias content: contentContainer.children

    Item {
        id: contentContainer
        Layout.preferredWidth: childrenRect.width
        Layout.preferredHeight: childrenRect.height
    }
}
