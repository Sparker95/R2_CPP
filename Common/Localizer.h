#pragma once

#include <vector>
#include <random>

#include "Pose.h"
#include "Map.h"
#include "Sensor.h"
#include "MathUtils.h"
#include "Chassis.h"

namespace Chassis
{
	class ChassisModel;
	struct ChassisInputs;
}

//#define LOCALIZER_STORE_HISTORY

class Localizer
{
public:
	enum class Mode
	{
		// Normal mode (full SLAM):
		// - Each particle has own map
		// - Weight is calculated from map of particle
		// - Map of particle is updated
		LOC_AND_MAP = 0,

		// Localization mode:
		// - Particle weight is calculated from common map,
		// - Map of particle is not updated
		COMMON_LOC = 1

	};

	struct Particle
	{
		Pose m_Pose; // Pose of car, not pose of sensor!
		float m_Weight = 0;
		OccupancyMap m_Map;
		Chassis::ChassisInputs m_InputsRnd; // Randomized input values used for this particle

#ifdef LOCALIZER_STORE_HISTORY
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
		float m_SensorPdfHalfWidth = 0.1f;
		float m_SensorPdfFloor = 0.5f;
		float m_SensorScanRange = 10.0f;
		float m_MapRes_w = 0.1f;
		Pose m_StartPose = {};
		uint m_nParticles = 32;
		int m_L2IntOccupiedCell = 8; // Use Utils::L2FloatToL2Int()
		int m_L2IntFreeCell = 8;
		int m_L2IntOccupiedCell_RT = 8; // RT - for RealTime map
		int m_L2IntFreeCell_RT = 8;
		uint m_ScanDecimation = 1;
	};

private:
	Chassis::ChassisModel* m_pChassisModel = nullptr;
	std::vector<Particle> m_Particles;
	OccupancyMap m_LocalizationCommonMap; // Map used in mode COMMON_LOC
	OccupancyMap m_RealTimeMap;
	const Config m_Config;
	std::minstd_rand m_RngResample;
	Pose m_MeanPose; // Recalculated from Update

	// Arrays reused during Update()
	std::vector<Particle> m_NewParticles;
	std::vector<uint> m_CopyOldParticles;
	std::vector<uint> m_OldParticlesCopyCount;
	std::vector<float> m_ExpectedScanDists;
	std::vector<Vector2> m_ScanDirections; // x, y - cos, sin of scan sample direction
	std::vector<Sensor::ScanSample> m_RealScanPolarFiltered; // Removed values too big or no-hits
	std::vector<Sensor::ScanSample> m_ScanPolarCompensated;

	// Diagnostics
	std::vector<HistoryEntry> m_History;
	uint m_PoseHistoryDivergeId = 0;       // Pose history diverges starting from this index
	std::vector<Pose> m_CommonPoseHistory; // Poses which are same in history of all particles

public:
	Localizer(const Config& config, Chassis::ChassisModel& statsModel);

	// Main functions
	void Update(const Chassis::ChassisInputs& inputs, const std::vector<Sensor::ScanSample>& realScanPolar, Mode mode);
	void Reset(); // Resets all particles and history
	void SwitchMode(Mode newMode);
private:
	void NormalizeWeights();
	void ResampleParticles();
	void UpdateMeanPose();

public:
	const std::vector<Particle>& GetParticles() const { return m_Particles; }
	std::vector<Particle>& GetParticles() { return m_Particles; }
	const Particle& GetMostLikelyParticle() const;
	const Pose& GetMeanPose() const { return m_MeanPose; }

	const OccupancyMap& GetLocalizationCommonMap() const { return m_LocalizationCommonMap; }
	const OccupancyMap& GetRealTimeMap() const { return m_RealTimeMap; }

	// Diagnostics
	const std::vector<HistoryEntry>& GetHistory() const { return m_History; }
	const std::vector<Pose>& GetCommonPoseHistory() const { return m_CommonPoseHistory; }

private:
	void GetScan(const Pose& sensorPose, const OccupancyMap& map, const std::vector<Vector2>& scanDirections, std::vector<float>& outScanDistances);
	void ApplyScan(const Pose& particleSensorPose, OccupancyMap& map, const std::vector<Vector2>& scanDirections, const std::vector<Sensor::ScanSample>& scanPolar, int l2FreeCell, int l2OccupiedCell);
	float CalculateParticleWeight(const Pose& particleSensorPose, const OccupancyMap& map, const std::vector<Vector2>& scanDirections, const std::vector<Sensor::ScanSample>& realScanPolar);
	float SensorRayPdf(float realDist, float expectedDist);

#ifdef LOCALIZER_STORE_HISTORY
	void StoreParticlesHistory(bool afterResampling);
#endif
};
