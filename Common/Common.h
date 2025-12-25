#pragma once

// Common definitions and types

// Full robot pose in the world
struct Pose
{
	float x = 0;
	float y = 0;
	float dir = 0;

	Pose() = default;
	Pose(float x, float y, float dir) : x(x), y(y), dir(dir) {}
};
