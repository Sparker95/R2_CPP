#include "Localizer.h"

#include <cmath>

#include "Scan.h"
#include "LineRasterizer.h"

Localizer::Localizer(uint nParticles, float sensorStdDev, float scanRange, float mapSizeWorld, float mapCellSize)
	: m_ScanRange(scanRange), m_MapCellSize(mapCellSize)
{
	int mapSize = static_cast<int>(ceilf(mapSizeWorld / mapCellSize));

	// Start pos is in the middle of the map
	Vector2 startPosWorld = Map::GetCellCenterPos(Map::CellPos(mapSize / 2, mapSize / 2));
	Pose startPose(startPosWorld.x, startPosWorld.y, 0.0f);

	m_Particles.reserve(nParticles);
	for (uint i = 0; i < nParticles; i++)
	{
		Particle p;
		p.m_Pose = startPose;
		p.m_Weight = 0;
		p.m_Map = Map(mapSize);

		m_Particles.push_back(p);
	}

	float a = -logf(sensorStdDev) - 0.5f * logf(PI_2X);
	m_SensorProbNoHitLog = logf(1.0f / m_ScanRange) - a;
	m_SensorPdfCoef = 1.0f / (2.0f * sensorStdDev * sensorStdDev);
}

void Localizer::Update(const float* realScanPolar)
{
	uint nParticles = static_cast<uint>(m_Particles.size());
	for (uint i = 0; i < nParticles; i++)
	{
		Particle& p = m_Particles[i];
		// Apply motion model to particles
		// WIP for testing
		{
			float moveDist = 0.23f;
			p.m_Pose.x += moveDist * std::cos(p.m_Pose.dir);
			p.m_Pose.y += moveDist * std::sin(p.m_Pose.dir);
		}
		
		// todo: reuse sin & cos values of scan angles

		// Calculate weight
		float weight = CalculateParticleWeight(p, realScanPolar);
		p.m_Weight = weight;

		// Update map
		ApplyScan(p.m_Pose, realScanPolar, Scan::SCAN_SIZE, m_ScanRange, p.m_Map, m_MapCellSize);
	}

	NormalizeWeights();
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

void Localizer::GetScan(const Pose& pose, float* outScanPolar, uint scanSize, float maxDistWorld, const Map& map, float mapCellSize)
{
	for (uint i = 0; i < scanSize; i++)
		outScanPolar[i] = Scan::SCAN_RAY_NO_HIT_VALUE;

	Vector2 posLocal(pose.x / mapCellSize, pose.y / mapCellSize);
	float maxDistLocal = maxDistWorld / mapCellSize;

	for (uint i = 0; i < scanSize; i++)
	{
		float lineDir = pose.dir + static_cast<float>(i) * PI_2X / static_cast<float>(scanSize);
		float lineDirx = std::cos(lineDir);
		float lineDiry = std::sin(lineDir);

		LineRasterizer lineRasterizer(posLocal.x, posLocal.y, lineDirx, lineDiry);

		while (true)
		{
			// Went beyond scan range
			if (lineRasterizer.GetDistance() > maxDistLocal)
				break;

			// Went beyond map bounds
			Map::CellPos mapCellPos = Map::GetCellIndex(lineRasterizer.GetPos());
			if (!map.IsCellPosWithinBounds(mapCellPos))
				break;

			Map::TCellData cellData = map.GetCellData(mapCellPos);

			// We've hit cell which is known to be occupied
			if (cellData < Map::CELL_VALUE_UNKNOWN)
			{
				outScanPolar[i] = mapCellSize * lineRasterizer.GetDistance();
				break;
			}

			// Advance one cell further
			++lineRasterizer;
		}
	}
}

void Localizer::ApplyScan(const Pose& pose, const float* scanPolar, uint scanSize, float maxDistWorld, Map& map, float mapCellSize)
{
	float invMapCellSize = 1.0f / mapCellSize;
	Vector2 posLocal(pose.x / mapCellSize, pose.y / mapCellSize);
	float maxDistLocal = maxDistWorld / mapCellSize;

	// How much cell data is decremented/incremented in case of hit/no hit
	const int cellDataInc = 40;

	for (uint i = 0; i < scanSize; i++)
	{
		float lineDir = pose.dir + static_cast<float>(i) * PI_2X / static_cast<float>(scanSize);
		float lineDirx = std::cos(lineDir);
		float lineDiry = std::sin(lineDir);

		LineRasterizer lineRasterizer(posLocal.x, posLocal.y, lineDirx, lineDiry);
		float rayDist = scanPolar[i];
		bool rayHit = rayDist != Scan::SCAN_RAY_NO_HIT_VALUE; // Did ray hit something?
		float rayDistLocal = rayHit ? invMapCellSize * rayDist : maxDistLocal;

		while (true)
		{
			Map::CellPos mapCellPos = Map::GetCellIndex(lineRasterizer.GetPos());

			if (!map.IsCellPosWithinBounds(mapCellPos))
				break;

			// Advance one cell further
			++lineRasterizer;

			// Note that mapCellPos is not current cell of lineRasterizer at this point, but previous one

			// If next cell is outside limit, resolve what to do with last cell
			if (lineRasterizer.GetDistance() > rayDistLocal)
			{
				// Set value of final cell
				if (rayHit) // If we have hit something
				{
					int cellData = static_cast<int>(map.GetCellData(mapCellPos));
					cellData -= cellDataInc;
					if (cellData < 0)
						cellData = 0;

					map.SetCellData(mapCellPos, static_cast<Map::TCellData>(cellData));
				}

				break;
			}

			// Next cell is not out of limit, this is free space
			int cellData = static_cast<int>(map.GetCellData(mapCellPos));
			cellData += cellDataInc;
			if (cellData >= static_cast<int>(Map::CELL_VALUE_MAX))
				cellData = Map::CELL_VALUE_MAX;
			map.SetCellData(mapCellPos, static_cast<Map::TCellData>(cellData));
		}
	}
}

float Localizer::CalculateParticleWeight(const Particle& p, const float* realScanPolar) const
{
	float expectedScanPolar[Scan::SCAN_SIZE];
	GetScan(p.m_Pose, expectedScanPolar, Scan::SCAN_SIZE, m_ScanRange, p.m_Map, m_MapCellSize);

	// Optimization is based on several points:
	// - Multiplication of normal distribution PDF is replaced with sum of natural logarithms.
	// - The scaling coefficient of normal distribution PDF is taken out of the loop and added to initial accumulator value.
	// - Therefore, in case of no-hit, the scaling coefficient is substracted from probability value of that ray.
	// - Finally the scaling coefficient is removed completely, since it doesn't affect ratio of particle weights in the end.

	float weightLog = 0.0f;
	for (uint i = 0; i < Scan::SCAN_SIZE; i++)
	{
		float expectedDist = expectedScanPolar[i];
		if (expectedDist == Scan::SCAN_RAY_NO_HIT_VALUE)
		{
			weightLog += m_SensorProbNoHitLog;
			continue;
		}
		float diff = expectedDist - realScanPolar[i];
		weightLog -= diff * diff * m_SensorPdfCoef;
	}

	return expf(weightLog);
}