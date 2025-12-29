#pragma once

#include <vector>
#include <random>

#include "Common.h"
#include "Map.h"
#include "Sensor.h"

namespace Chassis
{
	class ChassisStatsModel;
	struct ChassisInputs;
}

#define LOCALIZER_STORE_HISTORY 1

class Localizer
{
public:
	struct Particle
	{
		Pose m_Pose; // Pose of car, not pose of sensor!
		float m_Weight = 0;
		Map m_Map;

#if LOCALIZER_STORE_HISTORY
		std::vector<Pose> m_PoseHistory;
#endif

		Particle() = default;
	};

	struct HistoryEntry
	{
		struct ParticleEntry
		{
			Pose m_Pose;
			float m_Weight;
			std::vector<Pose> m_UniquePoseHistory;
		};

		std::vector<ParticleEntry> m_Particles;
		std::vector<Pose> m_CommonPoseHistory;
		uint m_Id = 0;
		bool m_bAfterResampling = false;
	};

	struct Config
	{
		float m_SensorPdfStdDev = 0.1f;
		float m_SensorPdfFloor = 0.1f;
		Pose m_SensorOffset_LS = {};
		float m_SensorScanRange = 5.0f;
		float m_MapRes_w = 0.125f;
		Pose m_StartPose = {};
		uint m_nParticles = 64;
		int m_L2IntOccupiedCell = 1; // Use Utils::L2FloatToL2Int()
		int m_L2IntFreeCell = 1;
		uint m_ScanDecimation = 1;
	};

private:
	Chassis::ChassisStatsModel* m_pChassisStatsModel = nullptr;
	std::vector<Particle> m_Particles;
	const Config m_Config;
	std::minstd_rand m_RngResample;

	// Arrays reused during Update()
	std::vector<Particle> m_NewParticles;
	std::vector<uint> m_CopyOldParticles;
	std::vector<uint> m_OldParticlesCopyCount;
	std::vector<float> m_ExpectedScanDists;
	//std::vector<Vector2Base<int>> m_LineCells;

	// Diagnostics
	std::vector<HistoryEntry> m_History;
	uint m_PoseHistoryDivergeId = 0;       // Pose history diverges starting from this index
	std::vector<Pose> m_CommonPoseHistory; // Poses which are same in history of all particles

public:
	Localizer(const Config& config, Chassis::ChassisStatsModel& statsModel);

	void Update(const Chassis::ChassisInputs& inputs, const std::vector<Sensor::ScanSample>& realScanPolar);
	void NormalizeWeights();
	void ResampleParticles();

	const std::vector<Particle>& GetParticles() const { return m_Particles; }
	std::vector<Particle>& GetParticles() { return m_Particles; }

	const Particle& GetMostLikelyParticle() const;

	// Diagnostics
	const std::vector<HistoryEntry>& GetHistory() const { return m_History; }
	const std::vector<Pose>& GetCommonPoseHistory() const { return m_CommonPoseHistory; }

private:
	void GetScan(const Pose& sensorPose, const Map& map, const std::vector<Sensor::ScanSample>& scanAngles, std::vector<float>& outScanDistances);
	void ApplyScan(const Pose& sensorPose, Map& map, std::vector<Sensor::ScanSample> scanPolar);
	float CalculateParticleWeight(const Particle& p, const std::vector<Sensor::ScanSample>& realScanPolar);
	float SensorRayPdf(float realDist, float expectedDist);

#if LOCALIZER_STORE_HISTORY
	void StoreParticlesHistory(bool afterResampling);
#endif
};
