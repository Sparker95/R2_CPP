#include "MathUtils.h"

#include "Vector2.h"

#include <cmath>

using uint = unsigned int;

uint MathUtils::CircleXLine(const Vector2& circleCenter, float r, const Vector2& l0, const Vector2& l1,
	Vector2* outIntersections)
{
	float ldx = l1.x - l0.x;
	float ldy = l1.y - l0.y;
	float dx = l0.x - circleCenter.x;
	float dy = l0.y - circleCenter.y;

	// Quadratic equation
	float a = ldx * ldx + ldy * ldy;
	float b = 2.0f * dx * ldx + 2.0f * dy * ldy;
	float c = dx * dx + dy * dy - r * r;

	// No solutions
	if (a == 0.0f)
		return 0;

	uint nSol = 0;
	float t[2] = { 0,0 };
	float D = b * b - 4.0f * a * c;
	if (D < 0.0f)
		return 0;
	else if (D == 0.0f)
	{
		nSol = 1;
		t[0] = -b / (2.0f * a);
	}
	else
	{
		nSol = 2;
		t[0] = (-b + std::sqrt(D)) / (2.0f * a);
		t[1] = (-b - std::sqrt(D)) / (2.0f * a);
	}

	// Return only intersections inside line segment
	uint nSolOut = 0;
	for (uint i = 0; i < nSol; i++)
	{
		if (t[i] < 0.0f || t[i] > 1.0f)
			continue;

		outIntersections[nSolOut] = l0 + Vector2(ldx, ldy) * t[i];
		nSolOut++;
	}
	return nSolOut;
}