#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include "../Common/Map.h"
#include "../Common/CommonIntTypes.h"

#include <QWidget>
#include <QList>

class MapWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapWidget(QWidget *parent = nullptr);

protected:
	Map m_Map;

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
	void SetMapData(Map& newMap);
	void SetMapData(Map&& newMap);

signals:
};

#endif // MAPWIDGET_H
