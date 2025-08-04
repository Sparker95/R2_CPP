#pragma once

#include <vector>

#include "Common.h"
#include "Map.h"

class Localizer
{
public:
	struct Particle
	{
		Pose m_Pose;
		float m_Weight = 0;
		Map m_Map;

		Particle() = default;
	};

private:
	std::vector<Particle> m_Particles;
	float m_SensorProbNoHitLog;
	float m_SensorPdfCoef;
	float m_ScanRange;
	
	float m_MapCellSize;

public:
	Localizer(uint nParticles, float sensorStdDev, float scanRange, float mapSizeWorld, float mapCellSize);

	void Update(const float* realScanPolar);

	void NormalizeWeights();

	const std::vector<Particle>& GetParticles() const { return m_Particles; }
	std::vector<Particle>& GetParticles() { return m_Particles; }


//private:
	static void GetScan(const Pose& pose, float* outScanPolar, uint scanSize, float maxDistWorld, const Map& map, float mapCellSize);
	static void ApplyScan(const Pose& pose, const float* scanPolar, uint scanSize, float maxDistWorld, Map& map, float mapCellSize);

private:
	float CalculateParticleWeight(const Particle& p, const float* realScanPolar) const;
};
