// r2perftest_win.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "Vector2.h"
#include "Scan.h"
#include "LineRasterizer.h"
#include "Map.h"
#include "MapUtils.h"
#include "Localizer.h"

using namespace std;

void TestMap()
{
	/*
	Map::MapConfig mapConfig;
	mapConfig.m_ResWorld = 0.05f;
	mapConfig.m_SizeWorld = 2.0f;
	Map map(mapConfig);

	MapUtils::FillCircle(&map, Vector2(0, 0), 0.3f, 0);
	MapUtils::FillRect(&map, Vector2(0.6f, 0.6f), Vector2(0.8f, 0.8f), 0);

	string str;
	MapUtils::MapToString(&map, str);
	cout << "Test map\n";
	cout << str;
	*/
}

void TestScanFromMap()
{
	float mapCellSize = 0.1f;
	float mapSizeWorld = 2.0f;
	int mapSize = static_cast<int>(mapSizeWorld / mapCellSize + 1.0f);
	Map map(mapSize);

	MapUtils::FillRect(map, mapCellSize, Vector2(0, 0), Vector2(1.5f, 1.5f), 0);
	MapUtils::FillRect(map, mapCellSize, Vector2(0.2f, 0.2f), Vector2(1.3f, 1.3f), 0xFF);
	MapUtils::FillRect(map, mapCellSize, Vector2(1.3f, 0.0f), Vector2(2.0f, 1.5f), 127); // Expose right wall

	string str;
	MapUtils::MapToString(&map, str);
	cout << "TestScanFromMap\n";
	cout << str;

	// Try get scan from map
	constexpr uint TEST_SCAN_SIZE = 2;
	float scan[TEST_SCAN_SIZE];
	Localizer::GetScan(Pose(0.6f, 0.6f, PI_HALF), scan, TEST_SCAN_SIZE, 10.0f, map, mapCellSize);
	cout << "Scan:\n";
	for (uint i = 0; i < TEST_SCAN_SIZE; i++)
	{
		cout << i << ": " << scan[i] << "\n";
	}
}

void TestApplyScanToMap()
{
	float mapCellSize = 0.1f;
	float mapSizeWorld = 2.0f;
	int mapSize = static_cast<int>(mapSizeWorld / mapCellSize + 1.0f);
	Map map(mapSize);

	constexpr uint scanSize = 128;
	float scan[scanSize];
	for (uint i = 0; i < scanSize; i++)
		scan[i] = 0.5f;

	Localizer::ApplyScan(Pose(0.0f, 0.0f, 0), scan, scanSize, 10, map, mapCellSize);

	string str;
	MapUtils::MapToString(&map, str);
	cout << "TestApplyScanToMap\n" << str;
}

void FillTestMap(Map& m, float cellSize)
{
	MapUtils::FillRect(m, cellSize, { 0, 0 }, { 6.0f, 8.0f }, Map::CELL_VALUE_MAX);

	const float w = 0.5f; // # Wall thickness
	MapUtils::FillRect(m, cellSize, { 0, 0 }, { w, 8.0f }, 0); // # 0
	MapUtils::FillRect(m, cellSize, { 0, 8.0f - w }, { 6.0f, 8.0f }, 0);
	MapUtils::FillRect(m, cellSize, { 6.0f - w, 0 }, { 6.0f, 8.0f }, 0);
	MapUtils::FillRect(m, cellSize, { 0, 0 }, { 6.0f, w }, 0);
	MapUtils::FillRect(m, cellSize, { 4.2f, 1 }, { 4.5f, 1.3f }, 0); // # 4
	MapUtils::FillRect(m, cellSize, { 3.0f, 3.3f }, { 3.3f, 4.3f }, 0);
	MapUtils::FillRect(m, cellSize, { 1.5f, 4.3f }, { 4.0f, 4.3f + w }, 0);
	MapUtils::FillRect(m, cellSize, { 2.0f, 4.3f + w }, { 2.0 + w, 5.6f }, 0);
	MapUtils::FillRect(m, cellSize, { 5.25f, 4.3f }, { 6.0f, 4.3f + w }, 0); // #8
	MapUtils::FillRect(m, cellSize, { 2.0f, 3.5f }, { 2.3f, 3.8f }, 0);
	MapUtils::FillRect(m, cellSize, { 2.8f, 3.8f }, { 3.0f, 4.0f }, 0);
	MapUtils::FillRect(m, cellSize, { 3.0f, 1.0f }, { 3.3f, 1.3f }, 0);

	MapUtils::FillCircle(m, cellSize, { 1.5f, 1.5f }, 0.5f, 0); // # 0
	MapUtils::FillCircle(m, cellSize, { 3.0f, 7.0f }, 0.25f, 0);
	MapUtils::FillCircle(m, cellSize, { 5.0f, 2.0f }, 0.1f, 0);
	MapUtils::FillCircle(m, cellSize, { 4.75f, 2.5f }, 0.1f, 0);
}

void TestLocalizerUpdate()
{
	float mapCellSize = 0.1f;
	float mapSizeWorld = 64.0f;
	int mapSize = static_cast<int>(mapSizeWorld / mapCellSize + 1.0f);
	Map map(mapSize);

	FillTestMap(map, mapCellSize);

	string str;
	MapUtils::MapToString(&map, str);
	cout << str;

	// Init localizer
	const float startx = 2.5f;
	const float starty = 2.5f;
	const float scanRange = 15.0f;
	const uint nParticles = 128;
	Localizer loc(nParticles, 1.5f, scanRange, mapSizeWorld, mapCellSize);

	auto& particles = loc.GetParticles();
	for (uint i = 0; i < particles.size(); i++)
	{
		Localizer::Particle& p = particles[i];
		p.m_Pose.x = startx;
		p.m_Pose.y = starty;
		p.m_Pose.dir = PI_2X * static_cast<float>(i) / static_cast<float>(particles.size());
		p.m_Map = map;
	}

	// Create a fake scan, take it from map and one of particles
	float scanPolar[Scan::SCAN_SIZE];
	Localizer::GetScan(particles[0].m_Pose, scanPolar, Scan::SCAN_SIZE, scanRange, map, mapCellSize);

	const uint nLocUpdates = 10;
	auto timestampStart = chrono::high_resolution_clock::now();
	{
		for (uint i = 0; i < nLocUpdates; i++)
			loc.Update(scanPolar);

		//std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	auto timestampEnd = chrono::high_resolution_clock::now();

	auto testDuration_us = chrono::duration_cast<chrono::microseconds>(timestampEnd - timestampStart);

	// Print particle weights
	cout << "Particle weights (normalized):\n";
	for (uint i = 0; i < particles.size(); i++)
	{
		cout << i << ": " << particles[i].m_Weight << "\n";
	}

	// Print duration
	float testDuration_ms = 1e-3f * static_cast<float>(testDuration_us.count());
	float testDuration_s = 1e-6f * static_cast<float>(testDuration_us.count());
	float updateDuration_ms = testDuration_ms / static_cast<float>(nLocUpdates);
	float updatesPerSecond = static_cast<float>(nLocUpdates) / testDuration_s;
	float particlesPerSecond = static_cast<float>(nParticles) * updatesPerSecond;
	cout << "N Particles:          " << nParticles << "\n";
	cout << "Map size:             " << mapSize << " x " << mapSize << ", " << mapSize*mapSize << " bytes\n";
	cout << "Test duration:        " << testDuration_ms << " ms\n";
	cout << "One loc. update:      " << updateDuration_ms << " ms\n";
	cout << "Updates per second:   " << updatesPerSecond << "\n";
	cout << "Particles per second: " << particlesPerSecond << "\n";
}

void TestRasterizer()
{
	cout << "Test line rasterizer\n";

	LineRasterizer r(0.31f, 0.31f,
		1.0f, 0.1f);

	int lx, ly;
	float dist;
	for (int i = 0; i < 10; i++)
	{
		dist = r.GetDistance();
		r.GetGridPos(lx, ly);

		cout << i << ": " << lx << ", " << ly << ", " << dist << "\n";

		++r;
	}
}

int main()
{
	//TestScanFromMap();
	//TestApplyScanToMap();
	//while (1)
		TestLocalizerUpdate();
}

