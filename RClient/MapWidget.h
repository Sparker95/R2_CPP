#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include "../Common/Map.h"
#include "../Common/Pose.h"
#include "../Common/Vector2.h"
#include "../Common/CommonIntTypes.h"

#include <QWidget>
#include <QList>

class MapWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapWidget(QWidget *parent = nullptr);

protected:
	// Data to display
	const Map* m_pMap = nullptr;
	QList<Pose> m_Poses;
	QList<Vector2> m_Path;
	QPointF m_CursorMarkerPos;
	bool m_CursorMarkerEnabled = false;

	QPointF m_ViewCenter_m; // center of view port in meters

	// Dragging
	bool m_Dragging = false;
	QPointF m_DragStartPos; // Cursor pos when dragging started
	QPointF m_DragStartViewCenter_m; // View center when dragging started

	// Zoom
	QList<qreal> m_ViewSizes_m;
	uint m_ViewSizeId; // Currently selected view size (zoom)

	void paintEvent(QPaintEvent *event) override;

	void mousePressEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
	void leaveEvent(QEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event);

public:
	void SetMapData(const Map* pMap);
	void SetLocationData(QList<Pose>& newPoses);
	void SetPathData(QList<Vector2>& newPath);

	bool GetCursorMarkerPos(QPointF& outPos) { outPos = m_CursorMarkerPos; return m_CursorMarkerEnabled; };

signals:
};

#endif // MAPWIDGET_H
