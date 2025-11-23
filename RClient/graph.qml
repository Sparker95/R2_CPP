import QtQuick 2.15
import QtGraphs

GraphsView {
    anchors.fill: parent
    axisX: ValueAxis {
        min: -3
        max: 3
    }
    axisY: ValueAxis {
        min: -3
        max: 3
    }

    component GraphMarker : Rectangle {
        width: 6
        height: 6
        color: "#ff0000"
        radius: width * 0.5
        border.width: 0
        border.color: "#000000"
    }

    ScatterSeries {
        color: "#00ff00"
        id: scatterSeries
        pointDelegate: GraphMarker { }
        XYPoint { x: 0; y: 0 }
    }
}
