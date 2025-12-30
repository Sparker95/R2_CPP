#pragma once

#include "Vector2.h"

#include <vector>

class LineRasterizer
{
private:
	float m_x;
	float m_y;
	float m_dist;
	float m_incx;
	float m_incy;
	float m_incDist; // Increment of distance with each step

public:
	LineRasterizer(float startx, float starty, float dirx, float diry);
	void operator++();
	Vector2 GetPos() { return Vector2(m_x, m_y); };

	float GetDistance() { return m_dist; }
};

namespace BresenhamLine
{
	void GetLine(Vector2Base<int> start, Vector2Base<int> end, std::vector<Vector2Base<int>>& outPoints);
}