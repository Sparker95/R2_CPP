#pragma once

#include "Vector2.h"

// Common math constants
constexpr static float PI = 3.1415926535897932384626433832795f;
constexpr static float PI_2X = 6.283185307179586476925286766559f;
constexpr static float PI_HALF = 1.5707963267948966192313216916398f;

namespace MathUtils
{
	using uint = unsigned int;

	// Circle and line segment intersection.
	// Returns count of intersections.
	uint CircleXLine(const Vector2& circleCenter, float r, const Vector2& l0, const Vector2& l1,
		Vector2* outIntersections);

	void ProjectPointOnLine(const Vector2& pt, const Vector2& l0, const Vector2& l1,
		float* outLineLen = nullptr, float* outProjDistFrom_l0 = nullptr, Vector2* outProjPoint = nullptr);
}