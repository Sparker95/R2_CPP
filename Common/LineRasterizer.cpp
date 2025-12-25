#include "LineRasterizer.h"

#include <cmath>

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
