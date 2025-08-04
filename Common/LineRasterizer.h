#pragma once

#include "Vector2.h"

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
	void GetGridPos(int& outx, int& outy); // todo remove later, world to grid conversions should happen through map
	Vector2 GetPos() { return Vector2(m_x, m_y); };

	float GetDistance() { return m_dist; }
};

