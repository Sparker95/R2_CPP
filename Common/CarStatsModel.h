#pragma once

#include <random>

#include "Common.h"

class CarStatsModel
{
private:
	float m_PosStdDevperDist = 0;
	float m_DirStdDevPerDist = 0;

	std::minstd_rand m_Rng;
	std::normal_distribution<float> m_Distribution;

public:
	CarStatsModel(float posStdDevperDist, float dirStdDevPerDist);

	Pose GetPoseSample(const Pose& poseWorld, const Pose& odoPoseDiffLocal);
};