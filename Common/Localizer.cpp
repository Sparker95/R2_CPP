#include "Localizer.h"

// Tracy
#include "Tracy/tracy/Tracy.hpp"

#include <cmath>

#include "Vector2.h"
#include "Utils.h"
#include "MathUtils.h"
#include "Scan.h"
#include "LineRasterizer.h"
//#include "CarStatsModel.h"

const unsigned int RNG_DEFAULT_SEED = 123;

Localizer::Localizer(const Config& config, Chassis::ChassisModel& statsModel)
	: m_pChassisModel(&statsModel), m_Config(config)
{
	m_Particles.reserve(config.m_nParticles);
	for (uint i = 0; i < config.m_nParticles; i++)
	{
		Particle p;
		p.m_Pose = config.m_StartPose;
		p.m_Weight = 0;
		p.m_Map = OccupancyMap(config.m_MapRes_w);

		m_Particles.push_back(p);
	}

	m_RngResample.seed(RNG_DEFAULT_SEED);
}


void Localizer::Reset()
{
	for (Particle& p : m_Particles)
	{
		p.m_Pose = m_Config.m_StartPose;
		p.m_Weight = 0;
		p.m_Map = OccupancyMap(m_Config.m_MapRes_w);

#ifdef LOCALIZER_STORE_HISTORY
		p.m_PoseHistory.clear();
#endif
	}
	m_LocalizationCommonMap = OccupancyMap(m_Config.m_MapRes_w);

	m_RngResample.seed(RNG_DEFAULT_SEED);

#ifdef LOCALIZER_STORE_HISTORY
	m_History.clear();
	m_PoseHistoryDivergeId = 0;
	m_CommonPoseHistory.clear();
#endif
}

void Localizer::SwitchMode(Localizer::Mode mode)
{
	if (mode == Localizer::Mode::LOC_AND_MAP)
	{
		// Copy map TO all particles
		for (Particle& p : m_Particles)
			p.m_Map = m_LocalizationCommonMap;
		m_LocalizationCommonMap.SetAllData(0);
	}
	else if (mode == Localizer::Mode::COMMON_LOC)
	{
		// From now localize based on map from most likely particle
		const Particle& mlp = GetMostLikelyParticle();
		m_LocalizationCommonMap = mlp.m_Map;

		// Maps of all particles get replaced with MLP map
		for (Particle& p : m_Particles)
			p.m_Map = m_LocalizationCommonMap;
	}
}

void Localizer::Update(const Chassis::ChassisInputs& inputs, const std::vector<Sensor::ScanSample>& realScanPolar, Mode mode)
{
	ZoneScopedN("Localizer::Update");

	// Filter the passed realScanPolar
	m_RealScanPolarFiltered.reserve(realScanPolar.size());
	m_RealScanPolarFiltered.clear();
	for (const auto& sample : realScanPolar)
	{
		if (sample.m_Dist_m == Scan::SCAN_RAY_NO_HIT_VALUE ||
			sample.m_Dist_m > m_Config.m_SensorScanRange)
			continue;
		m_RealScanPolarFiltered.push_back(sample);
	}

	if (inputs.m_OdoDiff == 0 || inputs.m_MoveState == Chassis::MoveState::STOP)
	{
		// No movement, almost everything remains same
		for (auto& p : m_Particles)
			p.m_InputsRnd = {};
		// Same weights, same mean pose, same maps, ...
	}
	else
	{
		// Regular update loop, some movement happened

		uint nParticles = static_cast<uint>(m_Particles.size());

		for (uint i = 0; i < nParticles; i++)
		{
			ZoneScopedN("Localizer::UpdateParticle");

			Particle& p = m_Particles[i];

			// Apply motion model to particle
			Pose randomPoseDiff_LS = m_pChassisModel->CalculateRandomPoseDiff_LS(inputs, p.m_InputsRnd);

			Pose poseRandomized = Utils::AddPoseDiff(p.m_Pose, randomPoseDiff_LS);
#ifdef LOCALIZER_STORE_HISTORY
			p.m_PoseHistory.push_back(p.m_Pose);
#endif
			p.m_Pose = poseRandomized;

			// Common data reused in futher calls
			uint nRealScanSamples = static_cast<uint>(m_RealScanPolarFiltered.size());
			const Chassis::ChassisConfig& chassisConfig = m_pChassisModel->GetChassisConfig();
			Pose particleSensorPose = Utils::AddPoseDiff(p.m_Pose, Pose(chassisConfig.m_sensorOffsetFwd, chassisConfig.m_sensorOffsetSide, chassisConfig.m_sensorOffsetDir_rad));
			m_ScanDirections.resize(m_RealScanPolarFiltered.size());
			for (uint sampleId = 0; sampleId < nRealScanSamples; sampleId++)
			{
				float sampleDir = particleSensorPose.dir + m_RealScanPolarFiltered[sampleId].m_Angle_rad;
				m_ScanDirections[sampleId].x = std::cos(sampleDir);
				m_ScanDirections[sampleId].y = std::sin(sampleDir);
			}

			// Compensate scan motion
			Scan::CompensateScanMotion(m_RealScanPolarFiltered, *m_pChassisModel, p.m_InputsRnd, m_ScanPolarCompensated);

			// Calculate weight. Which map to use depends on mode.
			const OccupancyMap& mapCalcWeight = mode == Mode::LOC_AND_MAP ? p.m_Map : m_LocalizationCommonMap;
			float weight = CalculateParticleWeight(particleSensorPose, mapCalcWeight, m_ScanDirections, m_ScanPolarCompensated);
			p.m_Weight = weight;

			// Update map - depends on mode.
			// Scan is applied at pose where Sensor is in World Space
			if (mode == Mode::LOC_AND_MAP)
				ApplyScan(particleSensorPose, p.m_Map, m_ScanDirections, m_ScanPolarCompensated,
					m_Config.m_L2IntFreeCell, m_Config.m_L2IntOccupiedCell);
		}

		NormalizeWeights();

		// Store history
#ifdef LOCALIZER_STORE_HISTORY
		StoreParticlesHistory(false);
#endif

		// Resample particles, use value of effective particles
		float sumWeightSq = 0.0f;
		for (uint i = 0; i < nParticles; i++)
			sumWeightSq += m_Particles[i].m_Weight * m_Particles[i].m_Weight;

		float nEff = 1 / sumWeightSq;

		if (nEff < 0.5f * static_cast<float>(nParticles))
		{
			//std::printf("N effective particles: %.2f, resampling\n", nEff);
			ResampleParticles();
		}
		else
		{
			//std::printf("N effective particles: %.2f, NO resampling\n", nEff);
		}

		// Store history
#ifdef LOCALIZER_STORE_HISTORY
		StoreParticlesHistory(true);
#endif

		UpdateMeanPose();
	}

	// Update of real time map
	{
		// Take most likely map, apply to it m_RealScanPolarFiltered with different weights

		const Particle& p = GetMostLikelyParticle();
		m_RealTimeMap = p.m_Map;

		// WIP this partically duplicates the main update algorithm
		uint nRealScanSamples = static_cast<uint>(m_RealScanPolarFiltered.size());
		const Chassis::ChassisConfig& chassisConfig = m_pChassisModel->GetChassisConfig();
		Pose particleSensorPose = Utils::AddPoseDiff(p.m_Pose, Pose(chassisConfig.m_sensorOffsetFwd, chassisConfig.m_sensorOffsetSide, chassisConfig.m_sensorOffsetDir_rad));
		m_ScanDirections.resize(m_RealScanPolarFiltered.size());
		for (uint sampleId = 0; sampleId < nRealScanSamples; sampleId++)
		{
			float sampleDir = particleSensorPose.dir + m_RealScanPolarFiltered[sampleId].m_Angle_rad;
			m_ScanDirections[sampleId].x = std::cos(sampleDir);
			m_ScanDirections[sampleId].y = std::sin(sampleDir);
		}

		// Compensate scan motion
		Scan::CompensateScanMotion(m_RealScanPolarFiltered, *m_pChassisModel, p.m_InputsRnd, m_ScanPolarCompensated);

		// Scan is applied to m_RealTimeMap!
		ApplyScan(particleSensorPose, m_RealTimeMap, m_ScanDirections, m_ScanPolarCompensated,
			m_Config.m_L2IntFreeCell_RT, m_Config.m_L2IntOccupiedCell_RT);
	}
}

void Localizer::NormalizeWeights()
{
	float sumWeights = 0.0f;
	uint nParticles = static_cast<uint>(m_Particles.size());
	for (uint i = 0; i < nParticles; i++)
		sumWeights += m_Particles[i].m_Weight;

	if (sumWeights == 0.0f)
	{
		float weight = 1.0f / static_cast<float>(nParticles);
		for (uint i = 0; i < nParticles; i++)
			m_Particles[i].m_Weight = weight;
		return;
	}

	float normCoeff = 1.0f / sumWeights;
	for (uint i = 0; i < nParticles; i++)
		m_Particles[i].m_Weight = normCoeff * m_Particles[i].m_Weight;
}

void Localizer::ResampleParticles()
{
	ZoneScopedN("Localizer::ResampleParticles");

	// Page 110
	uint nParticles = static_cast<uint>(m_Particles.size());
	std::uniform_real_distribution<float> distribution(0.0f, 1.0f / static_cast<float>(nParticles));
	float r = distribution(m_RngResample);
	float c = m_Particles[0].m_Weight; // Weight 0

	m_CopyOldParticles.clear();
	m_CopyOldParticles.resize(nParticles);
	m_OldParticlesCopyCount.clear();
	m_OldParticlesCopyCount.resize(nParticles, 0);

	{
		ZoneScopedN("Localizer::ResampleParticles SelectParticles");
		uint i = 0;
		for (uint m = 0; m < nParticles; m++)
		{
			float u = r + static_cast<float>(m) / static_cast<float>(nParticles);
			while (u > c)
			{
				i = (i + 1) % nParticles;
				c += m_Particles[i].m_Weight;
			}
			// Add particle i
			m_CopyOldParticles[m] = i;
			m_OldParticlesCopyCount[i]++;
		}
	}

	{
		ZoneScopedN("Localizer::ResampleParticles CopyParticles");

		m_NewParticles.clear();
		m_NewParticles.reserve(nParticles);
		for (uint i = 0; i < nParticles; i++)
		{
			uint iOldParticle = m_CopyOldParticles[i];
			Particle& oldParticle = m_Particles[iOldParticle];

			// If only one copy is left to be done, use move constructor, otherwise copy constructor
			if (m_OldParticlesCopyCount[iOldParticle] == 1)
				m_NewParticles.push_back(std::move(oldParticle));
			else
				m_NewParticles.push_back(oldParticle);

			m_OldParticlesCopyCount[iOldParticle]--;
		}
	}

	{
		{
			ZoneScopedN("Localizer::ResampleParticles SwapArrays");
			m_Particles.swap(m_NewParticles);
		}
		{
			ZoneScopedN("Localizer::ResampleParticles ClearNewParticles");
			m_NewParticles.clear();
		}
	}
}

void Localizer::UpdateMeanPose()
{
	NormalizeWeights();

	Vector2 meanPos;
	Vector2 meanDirVector;

	for (const Particle& p : m_Particles)
	{
		meanPos = meanPos + Vector2(p.m_Pose.x, p.m_Pose.y) * p.m_Weight;
		meanDirVector = meanDirVector + Vector2(std::cos(p.m_Pose.dir), std::sin(p.m_Pose.dir)) * p.m_Weight;
	}

	float meanDir = Utils::WrapAngleFull(std::atan2(meanDirVector.y, meanDirVector.x));
	m_MeanPose = Pose(meanPos.x, meanPos.y, meanDir);
}

void Localizer::GetScan(const Pose& sensorPose, const OccupancyMap& map, const std::vector<Vector2>& scanDirections, std::vector<float>& outScanDistances)
{
	ZoneScopedN("Localizer::GetScan");

	// Full polar scan is passed, but we need from it only angles.
	// This avoids extra allocation.

	uint nSamples = static_cast<uint>(scanDirections.size());
	outScanDistances.resize(nSamples);

	// _u - means Unscaled - means: 1.0 equals cell size
	Vector2 lineStart_w(sensorPose.x, sensorPose.y);
	Vector2 lineStart_u = lineStart_w / m_Config.m_MapRes_w;
	float scanRange_u = m_Config.m_SensorScanRange / m_Config.m_MapRes_w;

	for (uint i = 0; i < nSamples; i++)
	{
		float lineDirx = scanDirections[i].x;
		float lineDiry = scanDirections[i].y;

		LineRasterizer lineRasterizer(lineStart_u.x, lineStart_u.y, lineDirx, lineDiry);

		float hitDist_w = Scan::SCAN_RAY_NO_HIT_VALUE;
		while (true)
		{
			// Went beyond scan range
			if (lineRasterizer.GetDistance() > scanRange_u)
				break;

			// Went beyond map bounds
			Vector2 rasterizerPos = lineRasterizer.GetPos();
			OccupancyMap::TCellPos mapCellPos = map.PosWorldUnscaledToMap(rasterizerPos);
			if (!map.IsMapPosWithinBounds(mapCellPos))
				break;

			OccupancyMap::TCellData cellData = map.GetDataAtPos(mapCellPos);

			// We've hit cell which is known to be occupied
			if (cellData > 0)
			{
				// WIP find way to improve it, decide if map should store its resolution at all
				Vector2 hitPos_u = map.PosMapToWorldUnscaled(mapCellPos);
				hitDist_w = m_Config.m_MapRes_w * Vector2::Distance(lineStart_u, hitPos_u);
				// !!! Don't return distance to cell center.
				// !!! This often causes walls to align to horizontal or vertical cells.
				//hitDist_w = m_Config.m_MapRes_w * lineRasterizer.GetDistance();
				break;
			}

			// Advance one cell further
			++lineRasterizer;
		}

		outScanDistances[i] = hitDist_w;
	}
}

// Bresenham line variant, still has some problems
/*
void Localizer::GetScan(const Pose& sensorPose, const Map& map, const std::vector<Sensor::ScanSample>& scanAngles, std::vector<float>& outScanDistances)
{
	// Full polar scan is passed, but we need from it only angles.
	// This avoids extra allocation.

	uint nSamples = static_cast<uint>(scanAngles.size());
	outScanDistances.resize(nSamples);

	// _u - means Unscaled - means: 1.0 equals cell size
	Vector2 lineStart_w(sensorPose.x, sensorPose.y);
	Map::TCellPos mapStartPos = map.PosWorldToMap(lineStart_w);

	for (uint i = 0; i < nSamples; i++)
	{
		float lineDirAngle = sensorPose.dir + scanAngles[i].m_Angle_rad;
		Vector2 lineDir(std::cos(lineDirAngle), std::sin(lineDirAngle));
		Vector2 lineEnd_w = lineStart_w + lineDir * m_Config.m_SensorScanRange;
		Map::TCellPos mapEndPos = map.PosWorldToMap(lineEnd_w);

		BresenhamLine::GetLine(mapStartPos, mapEndPos, m_LineCells);

		float hitDist_w = Scan::SCAN_RAY_NO_HIT_VALUE;
		for (const auto& cellPos : m_LineCells)
		{
			// Went beyond map bounds
			if (!map.IsMapPosWithinBounds(cellPos))
				break;

			Map::TCellData cellData = map.GetDataAtPos(cellPos);

			// We've hit cell which is known to be occupied
			if (cellData > 0)
			{
				// Calculate dist to this grid pos
				Vector2 cellCenterPos_w = map.PosMapToWorld(cellPos);
				hitDist_w = Vector2::Distance(lineStart_w, cellCenterPos_w);
				break;
			}
		}

		outScanDistances[i] = hitDist_w;
	}
}
*/


void Localizer::ApplyScan(const Pose& sensorPose, OccupancyMap& map, const std::vector<Vector2>& scanDirections, const std::vector<Sensor::ScanSample>& scanPolar, int l2FreeCell, int l2OccupiedCell)
{
	ZoneScopedN("Localizer::ApplyScan");

	uint nSamples = static_cast<uint>(scanPolar.size());
	Vector2 lineStart_w(sensorPose.x, sensorPose.y);
	Vector2 lineStart_u = lineStart_w / m_Config.m_MapRes_w;

	// Find max distance of scan from actual data
	float scanMaxDist_w = scanPolar[0].m_Dist_m;
	for (uint i = 0; i < nSamples; i++)
	{
		if (scanPolar[i].m_Dist_m > scanMaxDist_w)
			scanMaxDist_w = scanPolar[i].m_Dist_m;
	}

	// Ensure map can represent the scan boundaries
	map.EnsureWorldBounds_Circle(lineStart_w, scanMaxDist_w + 16.0f * map.GetResolution_w()); // WIP find why it crashes, for now request more space

	for (uint i = 0; i < nSamples; i++)
	{
		Vector2 lineDir = scanDirections[i];

		LineRasterizer lineRasterizer(lineStart_u.x, lineStart_u.y, lineDir.x, lineDir.y);
		float rayDist_w = scanPolar[i].m_Dist_m;
		bool rayHit = rayDist_w != Scan::SCAN_RAY_NO_HIT_VALUE; // Did ray hit something?

		// No-hit rays are ignored
		if (!rayHit)
			continue;

		// _u means Unscaled
		float rayDist_u = rayDist_w / m_Config.m_MapRes_w;

		OccupancyMap::TCellPos lastMapCellPos = map.PosWorldUnscaledToMap(lineStart_u + lineDir * rayDist_u);

		while (true)
		{
			Vector2 rasterizerPos = lineRasterizer.GetPos();
			OccupancyMap::TCellPos mapCellPos = map.PosWorldUnscaledToMap(rasterizerPos);

			if (mapCellPos == lastMapCellPos)
				break;

			// Update cell as free space
			// No need to check if mapCellPos is within map bounds, this was ensured outside
			int cellData = static_cast<int>(map.GetDataAtPos(mapCellPos));
			cellData -= l2FreeCell;
			if (cellData <= -128)
				cellData = -128;
			map.SetDataAtPos(mapCellPos, static_cast<OccupancyMap::TCellData>(cellData));

			// Advance one cell further
			++lineRasterizer;

			// If next cell is outside limit, break
			if (lineRasterizer.GetDistance() > rayDist_u)
				break;
		}

		// Handle last cell
		int cellData = static_cast<int>(map.GetDataAtPos(lastMapCellPos));
		cellData += l2OccupiedCell;
		if (cellData >= 127)
			cellData = 127;

		map.SetDataAtPos(lastMapCellPos, static_cast<OccupancyMap::TCellData>(cellData));
	}
}


// Bresenham line variant, still has some problems
/*
void Localizer::ApplyScan(const Pose& sensorPose, Map& map, std::vector<Sensor::ScanSample> scanPolar)
{
	uint nSamples = static_cast<uint>(scanPolar.size());
	Vector2 lineStart_w(sensorPose.x, sensorPose.y);
	Map::TCellPos mapStartPos = map.PosWorldToMap(lineStart_w);

	// Find max distance of scan from actual data
	float scanMaxDist_w = scanPolar[0].m_Dist_m;
	for (uint i = 0; i < nSamples; i++)
	{
		if (scanPolar[i].m_Dist_m > scanMaxDist_w)
			scanMaxDist_w = scanPolar[i].m_Dist_m;
	}

	// Ensure map can represent the scan boundaries
	map.EnsureWorldBounds_Circle(lineStart_w, scanMaxDist_w);
	for (uint i = 0; i < nSamples; i++)
	{
		float rayDist_w = scanPolar[i].m_Dist_m;
		bool rayHit = rayDist_w != Scan::SCAN_RAY_NO_HIT_VALUE; // Did ray hit something?

		// No-hit rays are ignored
		if (!rayHit)
			continue;

		float lineDirAngle = sensorPose.dir + scanPolar[i].m_Angle_rad;
		Vector2 lineDir(std::cos(lineDirAngle), std::sin(lineDirAngle));
		Vector2 lineEnd_w = lineStart_w + lineDir * rayDist_w;

		Map::TCellPos mapEndPos = map.PosWorldToMap(lineEnd_w);
		BresenhamLine::GetLine(mapStartPos, mapEndPos, m_LineCells);

		uint nCellsInLine = static_cast<uint>(m_LineCells.size());
		for (uint cellId = 0; cellId < nCellsInLine - 1; cellId++) // All except last
		{
			// Update cell as free space
			// No need to check if mapCellPos is within map bounds, this was ensured outside
			Map::TCellPos cellPos = m_LineCells[cellId];
			int cellData = static_cast<int>(map.GetDataAtPos(cellPos));
			cellData -= m_Config.m_L2IntFreeCell;
			if (cellData <= -128)
				cellData = -128;
			map.SetDataAtPos(cellPos, static_cast<Map::TCellData>(cellData));
		}

		// Handle last cell
		int cellData = static_cast<int>(map.GetDataAtPos(mapEndPos));
		cellData += m_Config.m_L2IntOccupiedCell;
		if (cellData >= 127)
			cellData = 127;

		map.SetDataAtPos(mapEndPos, static_cast<Map::TCellData>(cellData));
	}
}
*/

float Localizer::CalculateParticleWeight(const Pose& particleSensorPose, const OccupancyMap& map, const std::vector<Vector2>& scanDirections, const std::vector<Sensor::ScanSample>& realScanPolar)
{
	ZoneScopedN("Localizer::CalculateParticleWeight");

	GetScan(particleSensorPose, map, scanDirections, m_ExpectedScanDists);

	float weight = std::pow(2.0f, 96.0f); // It's big number because weights only decrease.
	uint nSamples = static_cast<uint>(realScanPolar.size());
	for (uint i = 0; i < nSamples; i+= m_Config.m_ScanDecimation)
	{
		float expectedDist = m_ExpectedScanDists[i];
		float pdfValue = SensorRayPdf(realScanPolar[i].m_Dist_m, expectedDist);
		weight *= pdfValue;
	}

	return weight;
}

float Localizer::SensorRayPdf(float realDist, float expectedDist)
{
	if (expectedDist == Scan::SCAN_RAY_NO_HIT_VALUE)
		return m_Config.m_SensorPdfFloor;

	// https://www.desmos.com/calculator/oa2gg8rzuk
	float diff = realDist - expectedDist;
	float diffNorm = diff / m_Config.m_SensorPdfHalfWidth;
	/*
	float diffNormPow2 = diffNorm * diffNorm;
	float diffNormPow4 = diffNormPow2 * diffNormPow2;
	float diffNormPow8 = diffNormPow4 * diffNormPow4;
	float pdfNoFloor = 1.0f / (diffNormPow8 + 1.0f);
	return m_Config.m_SensorPdfFloor + (1.0f - m_Config.m_SensorPdfFloor) * pdfNoFloor;
	*/

	float diffNormAbs = std::abs(diffNorm);
	float diffNormAbsPow2 = diffNormAbs * diffNormAbs;
	float diffNormAbsPow4 = diffNormAbsPow2 * diffNormAbsPow2;
	float pdfNoFloor = std::exp2(-diffNormAbsPow4);
	return m_Config.m_SensorPdfFloor + (1.0f - m_Config.m_SensorPdfFloor) * pdfNoFloor;
}

const Localizer::Particle& Localizer::GetMostLikelyParticle() const
{
	const Particle* bestp = &m_Particles[0];
	float bestWeight = m_Particles[0].m_Weight;
	for (const Particle& p : m_Particles)
	{
		if (p.m_Weight > bestWeight)
		{
			bestWeight = p.m_Weight;
			bestp = &p;
		}
	}

	return *bestp;
}

#ifdef LOCALIZER_STORE_HISTORY
void Localizer::StoreParticlesHistory(bool afterResampling)
{
	ZoneScopedN("Localizer::StoreParticlesHistory");

	// Try to advance m_HistoryDivergeId
	uint poseHistorySize = static_cast<uint>(m_Particles[0].m_PoseHistory.size());
	while (m_PoseHistoryDivergeId < poseHistorySize)
	{
		// Increase index if all poses are now same
		const Pose& pose0 = m_Particles[0].m_PoseHistory[m_PoseHistoryDivergeId];
		bool allPosesEqual = true;
		for (const Particle& particle : m_Particles)
		{
			if (particle.m_PoseHistory[m_PoseHistoryDivergeId] != pose0)
			{
				allPosesEqual = false;
				break;
			}
		}
		if (allPosesEqual)
		{
			m_CommonPoseHistory.push_back(pose0);
			m_PoseHistoryDivergeId++;
		}
		else
			break;
	}

	m_History.push_back(HistoryEntry());
	HistoryEntry& historyEntry = m_History.back();
	historyEntry.m_bAfterResampling = afterResampling;
	historyEntry.m_Id = static_cast<uint>(m_History.size() - 1);
	historyEntry.m_CommonPoseHistory = m_CommonPoseHistory;
	for (const Particle& p : m_Particles)
	{
		historyEntry.m_Particles.push_back(HistoryEntry::ParticleEntry());
		HistoryEntry::ParticleEntry& particleEntry = historyEntry.m_Particles.back();
		particleEntry.m_Pose = p.m_Pose;
		particleEntry.m_Weight = p.m_Weight;

		particleEntry.m_UniquePoseHistory.reserve(p.m_PoseHistory.size() - m_CommonPoseHistory.size());

		uint copyId = static_cast<uint>(m_CommonPoseHistory.size());
		uint copyEnd = static_cast<uint>(p.m_PoseHistory.size());
		for (; copyId < copyEnd; copyId++)
			particleEntry.m_UniquePoseHistory.push_back(p.m_PoseHistory[copyId]);
	}
}
#endif