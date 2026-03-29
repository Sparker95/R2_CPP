#include "Utils.h"

#include <cmath>

#include "MathUtils.h"

float Utils::WrapAngleFull(float angle)
{
	while (angle < 0.0f)
		angle += PI_2X;
	while (angle > PI_2X)
		angle -= PI_2X;
	return angle;
}

float Utils::WrapAngleDiff(float angle)
{
	while (angle < -PI)
		angle += PI_2X;
	while (angle > PI)
		angle -= PI_2X;
	return angle;
}

Eigen::Matrix2f Utils::MRot(float angle)
{
	float _cos = std::cos(angle);
	float _sin = std::sin(angle);
	Eigen::Matrix2f m{
		{_cos, -_sin},
		{_sin, _cos}
	};
	return m;
}

Pose Utils::AddPoseDiff(const Pose& pose_WS, const Pose& poseDiff_LS)
{
	Eigen::Matrix2f mRot = MRot(pose_WS.dir);
	Eigen::Vector2f poseDiffXY_WS = mRot * Eigen::Vector2f(poseDiff_LS.x, poseDiff_LS.y);
	float newPoseDir = WrapAngleFull(pose_WS.dir + poseDiff_LS.dir);
	Pose newPose_WS(pose_WS.x + poseDiffXY_WS(0), pose_WS.y + poseDiffXY_WS(1), newPoseDir);
	return newPose_WS;
}

Vector2 Utils::PosWorldToLocal(const Pose& pose, const Vector2& pos_WS)
{
	Vector2 pos_centered(pos_WS.x - pose.x, pos_WS.y - pose.y);

	Eigen::Matrix2f mRot = MRot(-pose.dir);
	Eigen::Vector2f pos_LS = mRot * Eigen::Vector2f(pos_centered.x, pos_centered.y);

	return Vector2(pos_LS(0), pos_LS(1));
}

