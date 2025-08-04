#pragma once

// Common definitions and types
using uint = unsigned int;
using ushort = unsigned short;
using uchar = unsigned char;

// Common constants
constexpr static float PI = 3.1415926535897932384626433832795f;
constexpr static float PI_2X = 6.283185307179586476925286766559f;
constexpr static float PI_HALF = 1.5707963267948966192313216916398f;

// Full robot pose in the world
struct Pose
{
	float x = 0;
	float y = 0;
	float dir = 0;

	Pose() = default;
	Pose(float x, float y, float dir) : x(x), y(y), dir(dir) {}
};
