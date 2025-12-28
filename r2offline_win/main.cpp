// r2offline_win.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// Heap memory leak detection
// https://learn.microsoft.com/en-us/cpp/c-runtime-library/find-memory-leaks-using-the-crt-library?view=msvc-170
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include <cstdio>
#include <string>

#include "Chassis.h"
#include "Localizer.h"
#include "LocalizerUtils.h"
#include "CarStatsModel.h"
#include "IntTypes.h"
#include "Map.h"
#include "SensorStreamUtils.h"
#include "MapUtils.h"
#include "MathUtils.h"
#include "Utils.h"
#include "LineRasterizer.h"

void TestReadStreamFile(const char* pFilePath)
{
	printf("Reading stream file: %s\n", pFilePath);

	std::vector<SensorStreamBlock> blocks;
	bool readSuccess = SensorStreamUtils::ReadBlocksFromFile(pFilePath, blocks);

	if (!readSuccess)
	{
		printf("Failed to read SensorStreamBlocks from file!\n");
	}

	printf("Read blocks: %u\n", static_cast<uint>(blocks.size()));
}

void TestWriteMap(const char* pFilePath)
{
	Map map(0.25);
	MapUtils::FillRect(map, Vector2(-1, -1), Vector2(1, 1), 100);
	MapUtils::WriteMapToFile_L2Int(map, pFilePath, 8);
}

void TestCarStatsModel()
{
	CarStatsModel model(0.1f, 0.1f);

	for (uint i = 0; i < 10; i++)
	{
		Pose p(1.0f, 1.0f, PI);
		Pose poseDiffLocal(1.0f, 0.0f, 0.1f);
		Pose newPose = model.GetPoseSample(p, poseDiffLocal);
		printf("new pose x: %.3f, y: %.3f, dir: %.3f\n", newPose.x, newPose.y, newPose.dir);
	}
}

void TestBresenhamLine()
{
	using TCellPos = Vector2Base<int>;
	TCellPos start(50, 100);
	TCellPos end(100, 40);
	std::vector<TCellPos> points;
	BresenhamLine::GetLine(start, end, points);

	printf("Bresenham line from %i, %i to %i, %i:\n", start.x, start.y, end.x, end.y);
	for (const auto& p : points)
		printf("%i, %i\n", p.x, p.y);

}

void ProcessSensorStreamFile(const char* pInStreamFilePath, const char* pOutMapFilePath, const char* pOutLocHistoryPath)
{
	Chassis::ChassisConfig chassisConfig;
	Chassis::ChassisStatsConfig chassisStatsConfig;

	Pose startPose(0, 0, 0);
	
	//CarStatsModel carStatsModel(0.2f, 40.0f * PI / 180.0f);
	//CarStatsModel carStatsModel(0.2f, 40.0f * PI / 180.0f);
	//CarStatsModel carStatsModel(1e-5, 1e-5);

	Chassis::ChassisStatsModel chassisStatsModel(chassisConfig, chassisStatsConfig);

	Localizer::Config localizerConfig;
	localizerConfig.m_SensorStdDev = 0.6f;
	localizerConfig.m_SensorOffset_LS = Pose(chassisConfig.m_sensorOffsetFwd, chassisConfig.m_sensorOffsetSide, chassisConfig.m_sensorOffsetDir_rad);
	localizerConfig.m_SensorScanRange = 12.0f;
	localizerConfig.m_MapRes_w = 0.0625f;
	localizerConfig.m_StartPose = startPose;
	localizerConfig.m_nParticles = 128; // 128
	localizerConfig.m_L2IntOccupiedCell = Utils::L2FloatToL2Int(3.0f);
	localizerConfig.m_L2IntFreeCell = Utils::L2FloatToL2Int(0.5f);

	Localizer localizer(localizerConfig, chassisStatsModel);

	// Read all blocks
	std::vector<SensorStreamBlock> sensorStreamBlocks;
	if (!SensorStreamUtils::ReadBlocksFromFile(pInStreamFilePath, sensorStreamBlocks))
	{
		printf("Error: Failed to read sensor stream file\n");
		return;
	}

	// Process data
	SensorStreamBlock* pPrevBlock = &sensorStreamBlocks[0];
	for (uint blockId = 0; blockId < static_cast<uint>(sensorStreamBlocks.size()); blockId++)
	{
		printf("Processing block %u / %u\n", blockId, static_cast<uint>(sensorStreamBlocks.size()));
		SensorStreamBlock& block = sensorStreamBlocks[blockId];

		Chassis::TOdometerCounter odoDiff = Chassis::CalculateOdometerDifference(block.m_Odometer, pPrevBlock->m_Odometer);
		//Pose odoPoseDiff_LS = Chassis::CalculatePoseDiff_LS(chassisConfig, odoDiff, block.m_MoveState, block.m_Steer);

		if (odoDiff == 0)
		{
			pPrevBlock = &block;
			continue;
		}

		Chassis::ChassisInputs inputs;
		inputs.m_MoveState = block.m_MoveState;
		inputs.m_OdoDiff = odoDiff;
		inputs.m_Steer = block.m_Steer;
		localizer.Update(inputs, block.m_ScanSamples);

		pPrevBlock = &block;
	}

	// Write map
	printf("Finished processing data, writing results to files\n");
	const Localizer::Particle& bestParticle = localizer.GetMostLikelyParticle();
	MapUtils::WriteMapToFile_L2Int(bestParticle.m_Map, pOutMapFilePath, Utils::LOG2_PROB_QUANTIZATION);

	// Write particles history
	LocalizerUtils::WriteLocalizerHistoryToFile(localizer, pOutLocHistoryPath, 32);
}

int main(int argc, char* argv[])
{
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	if (argc == 1)
	{
		printf("Provide path to file!\n");
		return 1;
	}

	// Extract arguments
	const char* pInStreamFilePath = nullptr;
	const char* pOutMapFilePath = nullptr;
	const char* pOutLocHistoryPath = nullptr;
	for (int i = 1; i < argc - 1; i++)
	{
		std::string arg(argv[i]);
		if (arg == "-inStreamFile")
			pInStreamFilePath = argv[i + 1];
		else if (arg == "-outMapFile")
			pOutMapFilePath = argv[i + 1];
		else if (arg == "-outLocHistoryFile")
			pOutLocHistoryPath = argv[i + 1];
	}

	if (!pInStreamFilePath || !pOutMapFilePath || !pOutLocHistoryPath)
	{
		printf("Not all arguments are provided!\n");
		return 1;
	}

	printf("Arguments:\n");
	printf("-inStreamFile      : %s\n", pInStreamFilePath);
	printf("-outMapFile        : %s\n", pOutMapFilePath);
	printf("-outLocHistoryFile : %s\n", pOutLocHistoryPath);

	ProcessSensorStreamFile(pInStreamFilePath, pOutMapFilePath, pOutLocHistoryPath);

	//TestBresenhamLine();
	//const char* pFilePath = argv[1];
	//TestReadStreamFile(pFilePath);
	//TestWriteMap(pFilePath); // Warning: Don't enable both! File will be overridden.
	//TestCarStatsModel();

	_CrtDumpMemoryLeaks();

	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
