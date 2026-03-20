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

	static void RasterizeLine(Vector2Base<float> start, Vector2Base<float> end, std::vector<Vector2Base<int>>& outLine);

	// Horizontal span - part of scanline
	struct HSpan
	{
		int y;
		int xStart;
		int xEnd;
	};
	static void RasterizeRect(Vector2Base<float> center, Vector2Base<float> size, Vector2Base<float> dir, std::vector<HSpan>& outSpans);
};

namespace BresenhamLine
{
	// WIP This implementation seems broken, consider deleting
	void GetLine(Vector2Base<int> start, Vector2Base<int> end, std::vector<Vector2Base<int>>& outPoints);
}