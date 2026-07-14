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
	const OccupancyMap* m_pMap = nullptr;
	Pose m_MeanPose;
	QList<Pose> m_Poses;
	Vector2 m_PathDestPos = {};
	QList<Vector2> m_Path;
	QPointF m_PathPointPos;
	QPointF m_PathEndPointPos;
	QPointF m_PathLookaheadPos;
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
	void SetMapData(const OccupancyMap* pMap);
	void SetLocationData(QList<Pose>& newPoses);
	void SetMeanPose(const Pose& pose);
	void SetPathData(const Vector2& destPos, const QList<Vector2>& newPath);
	void SetPathFollowerData(const Vector2& pathPointPos, const Vector2& pathEndPointPos, const Vector2& lookaheadPos);

	bool GetCursorMarkerPos(QPointF& outPos) { outPos = m_CursorMarkerPos; return m_CursorMarkerEnabled; };

signals:
};

#endif // MAPWIDGET_H
