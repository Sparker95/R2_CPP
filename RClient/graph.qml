import QtQuick 2.15
import QtGraphs

GraphsView {
    anchors.fill: parent
    axisX: ValueAxis {
        min: -10
        max: 10
    }
    axisY: ValueAxis {
        min: -10
        max: 10
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
        XYPoint { x: 0.5; y: 0.5 }
        XYPoint { x: 1; y: 1 }
        XYPoint { x: 2; y: 2 }
        XYPoint { x: 2.5; y: 1.5 }
    }
}
