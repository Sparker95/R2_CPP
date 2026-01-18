#include "MapWidget.h"

#include <QPainter>
#include <QPainterStateGuard>
#include <QImage>
#include <QColor>
#include <QMouseEvent>
#include <QWheelEvent>

#include <stdint.h>
#include <algorithm>

#include "../Common/MapUtils.h"
#include "../Common/ProbUtils.h"

using uint = unsigned int;

MapWidget::MapWidget(QWidget *parent)
    : QWidget{parent}
{
	// Load a test map
	uint l2Quantization = 8;
	const char mapPath[] = "D:\\Projects\\Robot2\\CPP\\r2offline_win\\TestMap.rmap";
	MapUtils::ReadMapFromFile_L2Int(mapPath, m_Map, l2Quantization);

	// Load test poses
	m_Poses.reserve(256);
	for (uint i = 0; i < 256; i++)
	{
		uint rnd0 = (i * 789) % 255;
		uint rnd1 = (i * 1039) % 255;

		float x = static_cast<float>(rnd0) / 128.0f - 1.0f;
		float y = static_cast<float>(rnd1) / 128.0f - 1.0f;
		float dir = 0.0f;

		m_Poses.push_back(Pose(x, y, dir));
	}

	// Init m_ViewSizes_m
	m_ViewSizes_m.clear();
	qreal viewSize = 0.25;
	while (viewSize < 60.0)
	{
		m_ViewSizes_m.append(viewSize);
		viewSize *= 1.25;
	}
	m_ViewSizeId = m_ViewSizes_m.size() * 3 / 4;
}


void MapWidget::SetMapData(Map& newMap)
{
	m_Map = newMap;
	update();
}

void MapWidget::SetMapData(Map&& newMap)
{
	m_Map = newMap;
	update();
}

void MapWidget::SetLocationData(QList<Pose>& newPoses)
{
	m_Poses = newPoses;
	update();
}

void MapWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);

	const QColor bgColor(0, 0, 105);
	const QColor poseColor(255, 0, 128);

	QPainterStateGuard Johny(&painter);

	// Fill background
	painter.fillRect(0, 0, width(), height(), bgColor);

	// WIP
	Map::TCellSize mapSize = m_Map.GetSize();
	QImage img(QSize(mapSize.x, mapSize.y), QImage::Format::Format_Indexed8);
	QList<QRgb> colorTable;
	{
		colorTable.resize(256);
		for (uint colorId = 0; colorId < 256; colorId++)
		{
			int l2Int = static_cast<int>(static_cast<std::int8_t>(colorId));
			float l2Float = Utils::L2IntToL2Float(l2Int);
			float prob = Utils::L2FloatToProb(l2Float); // [0 .. 1.0]

			int rgb = std::clamp<int>(static_cast<int>(std::floor((1.0f - prob) * 255.0f)), 0, 255);

			colorTable[colorId] = QColor(rgb, rgb, rgb).rgb();
		}
	}
	img.setColorTable(colorTable);

	img.fill(100);

	for (int y = 0; y < img.height(); ++y)
	{
		uchar* imgLine = img.scanLine(y);
		Map::TCellData* mapLine = &m_Map.GetData()[y * mapSize.x];
		for (int x = 0; x < img.width(); ++x)
		{
			// WIP use memcpy
			imgLine[x] = static_cast<uchar>(mapLine[x]);
		}
	}


	// Scaling
	qreal viewSize_m = m_ViewSizes_m[m_ViewSizeId];
	qreal mapRes_w = m_Map.GetResolution_w();
	Map::TCellSize mapCenter = m_Map.GetCenter();
	qreal minWidthHeight = static_cast<qreal>(std::min<int>(width(), height()));
	qreal scaleWorldToView = minWidthHeight / viewSize_m;

	painter.translate(width() / 2, height() / 2); // Origin is in center
	painter.scale(scaleWorldToView, -scaleWorldToView); // Y axis is directed upwards

	// Now scale is in meters

	// Map is drawn centered at its mapCenter
	painter.translate(-m_ViewCenter_m.x(), -m_ViewCenter_m.y());
	{
		painter.save();
		painter.translate(-static_cast<qreal>(mapCenter.x) * mapRes_w, -static_cast<qreal>(mapCenter.y) * mapRes_w);
		painter.scale(mapRes_w, mapRes_w);
		painter.drawImage(QPoint(0, 0), img);
		painter.restore();
	}

	// Draw poses
	{
		QPen pen;
		pen.setCosmetic(true);
		pen.setColor(poseColor);
		pen.setWidth(3);
		painter.setPen(pen);

		for (const auto& p : static_cast<const QList<Pose>>(m_Poses))
		{
			painter.drawLine(QPointF(p.x, p.y), QPointF(p.x, p.y));
		}
	}

	// Map origin is indicated by two lines 1m in length (1m each direction from center)
	{
		// X axis
		QPen pen;
		pen.setCosmetic(true); // Ensures that scaling doesn't affect width
		pen.setColor(QColor(255, 0, 0)); // Red
		pen.setWidth(2);
		painter.setPen(pen);
		painter.drawLine(QPointF(-1, 0), QPointF(1.0, 0));

		// Y axis
		pen.setColor(QColor(0, 255, 0)); // Green
		painter.setPen(pen);
		painter.drawLine(QPointF(0, -1), QPointF(0, 1.0));
	}
}

void MapWidget::mousePressEvent(QMouseEvent *event)
{
	event->accept();
	m_Dragging = true;
	m_DragStartPos = event->position();
	m_DragStartViewCenter_m = m_ViewCenter_m;
}

void MapWidget::mouseReleaseEvent(QMouseEvent *event)
{
	event->accept();
	m_Dragging = false;
}

void MapWidget::leaveEvent(QEvent *event)
{
	event->accept();
	m_Dragging = false;
}

void MapWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (!m_Dragging)
		return;

	event->accept();
	QPointF dragOffset_px = event->position() - m_DragStartPos;

	qreal minWidthHeight = static_cast<qreal>(std::min<int>(width(), height()));
	qreal scaleViewToWorld = m_ViewSizes_m[m_ViewSizeId] / minWidthHeight;

	QPointF dragOffset_m = dragOffset_px * scaleViewToWorld;
	dragOffset_m.setY(-dragOffset_m.y()); // Y is inverted relative to screen
	m_ViewCenter_m = m_DragStartViewCenter_m - dragOffset_m;

	update();
}

void MapWidget::wheelEvent(QWheelEvent *event)
{
	bool wheelDir = event->angleDelta().y() > 0;

	if (wheelDir)
	{
		if (m_ViewSizeId != 0)
			m_ViewSizeId--;
	}
	else
		m_ViewSizeId++;

	m_ViewSizeId = std::clamp<uint>(m_ViewSizeId, 0, m_ViewSizes_m.size() - 1);

	update();
}
