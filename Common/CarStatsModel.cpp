#include "CarStatsModel.h"

#include <cmath>

#include "Utils.h"

CarStatsModel::CarStatsModel(float posStdDevperDist, float dirStdDevPerDist) :
	m_PosStdDevperDist(posStdDevperDist),
	m_DirStdDevPerDist(dirStdDevPerDist)
{
	m_Rng.seed(666);
}

Pose CarStatsModel::GetPoseSample(const Pose& poseWorld, const Pose& odoPoseDiffLocal)
{
	using TDistrParam = std::normal_distribution<float>::param_type;

	float dist = std::sqrtf(odoPoseDiffLocal.x * odoPoseDiffLocal.x +
		odoPoseDiffLocal.y * odoPoseDiffLocal.y);

	float posStdDev = dist * m_PosStdDevperDist;
	float dirStdDev = dist * m_DirStdDevPerDist;

	TDistrParam distrParamPos(0.0f, posStdDev);
	float rndx = m_Distribution(m_Rng, distrParamPos);
	float rndy = m_Distribution(m_Rng, distrParamPos);
	
	TDistrParam distrParamDir(0.0f, dirStdDev);
	float rndDir = m_Distribution(m_Rng, distrParamDir);

	// Rotation matrix
	Eigen::Matrix2f mRot = Utils::MRot(poseWorld.dir);
	Eigen::Vector2f poseDiffLocalRandomizedXY(odoPoseDiffLocal.x + rndx,
		odoPoseDiffLocal.y + rndy);
	Eigen::Vector2f poseDiffWorldXY = mRot * poseDiffLocalRandomizedXY;

	Pose newPose(poseWorld.x + poseDiffWorldXY(0),
		poseWorld.y + poseDiffWorldXY(1),
		Utils::WrapAngleFull(poseWorld.dir + odoPoseDiffLocal.dir + rndy)
	);

	return newPose;
}
