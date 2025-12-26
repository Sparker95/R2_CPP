#include "Localizer.h"

#include <cmath>

#include "Utils.h"
#include "MathUtils.h"
#include "Scan.h"
#include "LineRasterizer.h"
#include "CarStatsModel.h"

Localizer::Localizer(const Config& config, CarStatsModel& carStatsModel)
	: m_Config(config), m_pCarStatsModel(&carStatsModel)
{
	m_Particles.reserve(config.m_nParticles);
	for (uint i = 0; i < config.m_nParticles; i++)
	{
		Particle p;
		p.m_Pose = config.m_StartPose;
		p.m_Weight = 0;
		p.m_Map = Map(config.m_MapRes_w);

		m_Particles.push_back(p);
	}

	m_RngResample.seed(123);

	float a = -logf(m_Config.m_SensorStdDev) - 0.5f * logf(PI_2X);
	m_SensorProbNoHitLog = logf(1.0f / m_Config.m_SensorScanRange) - a;
	m_SensorPdfCoef = 1.0f / (2.0f * m_Config.m_SensorStdDev * m_Config.m_SensorStdDev);
}

void Localizer::Update(Pose odoPoseDiffLocal, const std::vector<Sensor::ScanSample>& realScanPolar)
{
	uint nParticles = static_cast<uint>(m_Particles.size());

	for (uint i = 0; i < nParticles; i++)
	{
		Particle& p = m_Particles[i];

		// Apply motion model to particle
		Pose poseRandomized = m_pCarStatsModel->GetPoseSample(p.m_Pose, odoPoseDiffLocal);
		// WIP Push history of this particle
		p.m_Pose = poseRandomized;

		// Calculate weight
		float weight = CalculateParticleWeight(p, realScanPolar);
		p.m_Weight = weight;

		// Update map
		// Scan is applied at pose where Sensor is in World Space
		Pose sensorPoseRandomized = Utils::AddPoseDiff(poseRandomized, m_Config.m_SensorOffset_LS);
		ApplyScan(sensorPoseRandomized, p.m_Map, realScanPolar);
	}

	NormalizeWeights();

	// Resample particles, use value of effective particles
	float sumWeightSq = 0.0f;
	for (uint i = 0; i < nParticles; i++)
		sumWeightSq += m_Particles[i].m_Weight * m_Particles[i].m_Weight;

	float nEff = 1 / sumWeightSq;

	if (nEff < 0.5f * static_cast<float>(nParticles))
	{
		std::printf("N effective particles: %.2f, resampling\n", nEff);
		ResampleParticles();
	}
	else
		std::printf("N effective particles: %.2f, NO resampling\n", nEff);
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
	// Page 110
	uint nParticles = static_cast<uint>(m_Particles.size());
	std::uniform_real_distribution<float> distribution(0.0f, 1.0f / static_cast<float>(nParticles));
	float r = distribution(m_RngResample);
	float c = m_Particles[0].m_Weight; // Weight 0

	m_CopyOldParticles.clear();
	m_CopyOldParticles.resize(nParticles);
	m_OldParticlesCopyCount.clear();
	m_OldParticlesCopyCount.resize(nParticles, 0);

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

	m_Particles.swap(m_NewParticles);
	m_NewParticles.clear();
}


void Localizer::GetScan(const Pose& sensorPose, const Map& map, const std::vector<Sensor::ScanSample>& scanAngles, std::vector<float>& outScanDistances)
{
	// Full polar scan is passed, but we need from it only angles.
	// This avoids extra allocation.

	uint nSamples = static_cast<uint>(scanAngles.size());
	outScanDistances.resize(nSamples);

	// _u - means Unscaled - means: 1.0 equals cell size
	Vector2 lineStart_w(sensorPose.x, sensorPose.y);
	Vector2 lineStart_u = lineStart_w / m_Config.m_MapRes_w;
	float scanRange_u = m_Config.m_SensorScanRange / m_Config.m_MapRes_w;

	for (uint i = 0; i < nSamples; i++)
	{
		float lineDir = sensorPose.dir + scanAngles[i].m_Angle_rad;
		float lineDirx = std::cos(lineDir);
		float lineDiry = std::sin(lineDir);

		LineRasterizer lineRasterizer(lineStart_u.x, lineStart_u.y, lineDirx, lineDiry);

		float hitDist_w = Scan::SCAN_RAY_NO_HIT_VALUE;
		while (true)
		{
			// Went beyond scan range
			if (lineRasterizer.GetDistance() > scanRange_u)
				break;

			// Went beyond map bounds
			Vector2 rasterizerPos = lineRasterizer.GetPos();
			Map::TCellPos mapCellPos = map.PosWorldUnscaledToMap(rasterizerPos);
			if (!map.IsMapPosWithinBounds(mapCellPos))
				break;

			Map::TCellData cellData = map.GetDataAtPos(mapCellPos);

			// We've hit cell which is known to be occupied
			if (cellData > 0)
			{
				// WIP find way to improve it, decide if map should store its resolution at all
				Vector2 hitPos_u = map.PosMapToWorldUnscaled(mapCellPos);
				hitDist_w = m_Config.m_MapRes_w * Vector2::Distance(lineStart_u, hitPos_u);
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


void Localizer::ApplyScan(const Pose& sensorPose, Map& map, std::vector<Sensor::ScanSample> scanPolar)
{
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
	map.EnsureWorldBounds_Circle(lineStart_w, scanMaxDist_w);

	for (uint i = 0; i < nSamples; i++)
	{
		float lineDirAngle = sensorPose.dir + scanPolar[i].m_Angle_rad;
		Vector2 lineDir(std::cos(lineDirAngle), std::sin(lineDirAngle));

		LineRasterizer lineRasterizer(lineStart_u.x, lineStart_u.y, lineDir.x, lineDir.y);
		float rayDist_w = scanPolar[i].m_Dist_m;
		bool rayHit = rayDist_w != Scan::SCAN_RAY_NO_HIT_VALUE; // Did ray hit something?

		// No-hit rays are ignored
		if (!rayHit)
			continue;

		// _u means Unscaled
		float rayDist_u = rayDist_w / m_Config.m_MapRes_w;

		Map::TCellPos lastMapCellPos = map.PosWorldUnscaledToMap(lineStart_u + lineDir * rayDist_u);

		while (true)
		{
			Vector2 rasterizerPos = lineRasterizer.GetPos();
			Map::TCellPos mapCellPos = map.PosWorldUnscaledToMap(rasterizerPos);

			if (mapCellPos == lastMapCellPos)
				break;

			// Update cell as free space
			// No need to check if mapCellPos is within map bounds, this was ensured outside
			int cellData = static_cast<int>(map.GetDataAtPos(mapCellPos));
			cellData -= m_Config.m_L2IntFreeCell;
			if (cellData <= -128)
				cellData = -128;
			map.SetDataAtPos(mapCellPos, static_cast<Map::TCellData>(cellData));

			// Advance one cell further
			++lineRasterizer;

			// If next cell is outside limit, break
			if (lineRasterizer.GetDistance() > rayDist_u)
				break;
		}

		// Handle last cell
		int cellData = static_cast<int>(map.GetDataAtPos(lastMapCellPos));
		cellData += m_Config.m_L2IntOccupiedCell;
		if (cellData >= 127)
			cellData = 127;

		map.SetDataAtPos(lastMapCellPos, static_cast<Map::TCellData>(cellData));
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

float Localizer::CalculateParticleWeight(const Particle& p, const std::vector<Sensor::ScanSample>& realScanPolar)
{
	Pose particleSensorPose = Utils::AddPoseDiff(p.m_Pose, m_Config.m_SensorOffset_LS);
	GetScan(particleSensorPose, p.m_Map, realScanPolar, m_ExpectedScanDists);

	// Optimization is based on several points:
	// - Multiplication of normal distribution PDF is replaced with sum of natural logarithms.
	// - The scaling coefficient of normal distribution PDF is taken out of the loop and added to initial accumulator value.
	// - Therefore, in case of no-hit, the scaling coefficient is substracted from probability value of that ray.
	// - Finally the scaling coefficient is removed completely, since it doesn't affect ratio of particle weights in the end.

	float weightLog = 0.0f;
	uint nSamples = static_cast<uint>(realScanPolar.size());
	for (uint i = 0; i < nSamples; i++)
	{
		float expectedDist = m_ExpectedScanDists[i];
		if (expectedDist == Scan::SCAN_RAY_NO_HIT_VALUE)
		{
			weightLog += m_SensorProbNoHitLog;
			continue;
		}
		float diff = realScanPolar[i].m_Dist_m - expectedDist;
		weightLog -= diff * diff * m_SensorPdfCoef;
	}

	return expf(weightLog);
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