#include "LineRasterizer.h"

#include <cmath>
#include <algorithm>

using uint = unsigned int;

LineRasterizer::LineRasterizer(float startx, float starty, float dirx, float diry)
	: m_x(startx), m_y(starty), m_dist(0)
{
	if (std::fabs(dirx) > std::fabs(diry))
	{
		m_incx = std::copysign(1.0f, dirx);
		m_incy = std::copysign(diry / dirx, diry);
	}
	else
	{
		m_incx = std::copysign(dirx / diry, dirx);
		m_incy = std::copysign(1.0f, diry);
	}

	m_incDist = std::sqrt(m_incx * m_incx + m_incy * m_incy);
}

void LineRasterizer::operator++()
{
	m_x += m_incx;
	m_y += m_incy;
	m_dist += m_incDist;
}

void LineRasterizer::RasterizeLine(Vector2Base<float> start, Vector2Base<float> end, std::vector<Vector2Base<int>>& outLine)
{
	Vector2Base<float> dir = end - start;
	float dirx = dir.x;
	float diry = dir.y;
	float lineLength = std::sqrt(dirx * dirx + diry * diry);
	dirx /= lineLength;
	diry /= lineLength;
	LineRasterizer r(start.x, start.y, dirx, diry);

	outLine.clear();
	outLine.reserve(static_cast<unsigned int>(lineLength) + 2);

	while (true)
	{
		if (r.GetDistance() < lineLength)
		{
			Vector2 pos = r.GetPos();
			outLine.push_back({ static_cast<int>(std::floor(pos.x)), static_cast<int>(std::floor(pos.y)) });
			++r;
		}
		else
		{
			// Check if last pixel should be added
			Vector2Base<int> endCell({ static_cast<int>(std::floor(end.x)), static_cast<int>(std::floor(end.y)) });

			if (!outLine.empty())
			{
				Vector2Base<int>& prevCell = outLine.back();
				if (!(prevCell == endCell))
					outLine.push_back(endCell);
			}
			else
				outLine.push_back(endCell);

			break;
		}
	}
}

void LineRasterizer::RasterizeRect(Vector2Base<float> center, Vector2Base<float> size, Vector2Base<float> dir, std::vector<LineRasterizer::HSpan>& outSpans)
{
	using Vector = Vector2Base<float>;

	Vector dirl(dir.x, dir.y);  // Direction along length
	Vector dirw(dir.y, -dir.x); // Direction along width

	float hw = 0.5f * size.x; // Half width
	float hl = 0.5f * size.y; // Half height

	// Calculate coordinates of all corners
	Vector cornerSigns[5] = {
		{1,  1},
		{1,  -1},
		{-1, -1},
		{-1, 1},
		{1,  1}
	};
	Vector corners[5];

	for (uint cornerId = 0; cornerId < 5; cornerId++)
	{
		Vector& cs = cornerSigns[cornerId];
		Vector corner = center + dirl * cs.x * hl + dirw * cs.y * hw;
		corners[cornerId] = corner;
	}

	// Rasterize lines between all corners
	std::vector<Vector2Base<int>> allBorderCells;
	allBorderCells.reserve(static_cast<uint>(3.0f * (size.x + size.y + 4.0))); // Rough estimate

	std::vector<Vector2Base<int>> borderLineCells; // WIP could be eliminated if RastLine didn't clear the out array
	for (uint cornerId = 0; cornerId < 4; cornerId++)
	{
		uint nextCornerId = cornerId + 1;

		RasterizeLine(corners[cornerId], corners[nextCornerId], borderLineCells);
		for (const auto& c : borderLineCells)
			allBorderCells.push_back(c);
	}

	// Find min, max x,y of border cells
	int minx = allBorderCells[0].x;
	int maxx = minx;
	int miny = allBorderCells[0].y;
	int maxy = miny;
	for (const auto& cell : allBorderCells)
	{
		int x = cell.x;
		int y = cell.y;
		if (x < minx)
			minx = x;
		if (x > maxx)
			maxx = x;
		if (y < miny)
			miny = y;
		if (y > maxy)
			maxy = y;
	}

	// Create horizontal spans
	int nHorizLines = maxy - miny + 1;
	std::vector<int> spanMin(nHorizLines, maxx);
	std::vector<int> spanMax(nHorizLines, minx);

	for (const auto& cell : allBorderCells)
	{
		int x = cell.x;
		int y = cell.y;
		int vertId = y - miny;

		if (x < spanMin[vertId])
			spanMin[vertId] = x;
		if (x > spanMax[vertId])
			spanMax[vertId] = x;
	}

	// Return horizontal spans: list of (y, xStart, xEnd)
	outSpans.clear();
	outSpans.resize(nHorizLines);
	for (int lineId = 0; lineId < nHorizLines; lineId++)
	{
		HSpan& span = outSpans[lineId];
		span.y = lineId + miny;
		span.xStart = spanMin[lineId];
		span.xEnd = spanMax[lineId];
	}
}

void BresenhamLine::GetLine(Vector2Base<int> start, Vector2Base<int> end, std::vector<Vector2Base<int>>& outPoints)
{
	// https://github.com/Azrood/python-bresenham/blob/master/bresenham_line.py
	// Setup initial conditions
	int x1 = start.x;
	int y1 = start.y;
	int x2 = end.x;
	int y2 = end.y;
	int dx = x2 - x1;
	int dy = y2 - y1;

	// Determine how steep the line is
	bool is_steep = std::abs(dy) > std::abs(dx);

	//  Rotate line
	if (is_steep)
	{
		std::swap(x1, y1);
		std::swap(x2, y2);
	}

	// Swap start and end points if necessary and store swap state
	bool swapped = false;
	if (x1 > x2)
	{
		std::swap(x1, x2);
		std::swap(y1, y2);
		swapped = true;
	}

	// Recalculate differentials
	dx = x2 - x1;
	dy = y2 - y1;

	// Calculate error
	int error = static_cast<int>(static_cast<float>(dx) * 0.5f);
	int ystep = y1 < y2 ? 1 : -1;

	// Iterate over bounding box generating points between start and end
	int y = y1;
	outPoints.clear();
	outPoints.reserve(dx);
	for (int x = x1; x <= x2; x++)
	{
		Vector2Base<int> coord = is_steep ? Vector2Base<int>(y, x) : Vector2Base<int>(x, y);
		outPoints.push_back(coord);
		error -= std::abs(dy);
		if (error < 0)
		{
			y += ystep;
			error += dx;
		}
	}

	// Reverse the list if the coordinates were swapped
	if (swapped)
		std::reverse(outPoints.begin(), outPoints.end());
}
