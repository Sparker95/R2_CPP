#pragma once

#include "Pose.h"
#include "Vector2.h"

#include <Eigen/Core>

namespace Utils
{
	// Angle utils
	// Returns angle within 0..2*pi
	float WrapAngleFull(float angle);

	// Returns angle within -pi..pi
	float WrapAngleDiff(float angle);

	// Rotation matrix
	Eigen::Matrix2f MRot(float angle);

	// Adds a LS pose diff to WS pose
	Pose AddPoseDiff(const Pose& pose_WS, const Pose& poseDiff_LS);

	// Converts point position from WS to LS of the pose
	Vector2 PosWorldToLocal(const Pose& pose, const Vector2& pos_WS);
}
