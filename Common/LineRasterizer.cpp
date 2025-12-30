#include "LineRasterizer.h"

#include <cmath>
#include <algorithm>

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
