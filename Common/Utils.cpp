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

void Utils::ScanSensorToMatrix(const Sensor::ScanSample* pSamples, uint scanSize, Scan::TScanPolar& outScanPolar)
{
	outScanPolar.resize(Eigen::NoChange, scanSize);
	for (uint i = 0; i < scanSize; i++)
	{
		outScanPolar(0, i) = pSamples[i].m_Angle_rad;
		outScanPolar(1, i) = pSamples[i].m_Dist_m;
	}
}

