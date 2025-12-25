// r2offline_win.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <cstdio>
#include <string>

#include "Chassis.h"
#include "Localizer.h"
#include "CarStatsModel.h"
#include "IntTypes.h"
#include "Map.h"
#include "SensorStreamUtils.h"
#include "MapUtils.h"
#include "MathUtils.h"
#include "Utils.h"

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

void ProcessSensorStreamFile(const char* pInStreamFilePath, const char* pOutMapFilePath)
{
	Chassis::ChassisConfig chassisConfig;
	
	Pose startPose(0, 0, 0);
	
	CarStatsModel carStatsModel(0.2f, 40.0f * PI / 180.0f);
	
	Localizer::Config localizerConfig;
	localizerConfig.m_SensorStdDev = 0.8f;
	localizerConfig.m_SensorOffset_LS = Pose(chassisConfig.m_sensorOffsetFwd, chassisConfig.m_sensorOffsetSide, chassisConfig.m_sensorOffsetDir_rad);
	localizerConfig.m_SensorScanRange = 12.0f;
	localizerConfig.m_MapRes_w = 0.0625f;
	localizerConfig.m_StartPose = startPose;
	localizerConfig.m_nParticles = 128;
	localizerConfig.m_L2IntOccupiedCell = Utils::L2FloatToL2Int(3.0f);
	localizerConfig.m_L2IntFreeCell = Utils::L2FloatToL2Int(0.3f);

	Localizer localizer(localizerConfig, carStatsModel);

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
		Pose odoPoseDiff_LS = Chassis::CalculatePoseDiff_LS(chassisConfig, odoDiff, block.m_MoveState, block.m_Steer);

		if (odoPoseDiff_LS.x == 0 && odoPoseDiff_LS.y == 0 /*& odoPoseDiff_LS.dir == 0*/)
		{
			pPrevBlock = &block;
			continue;
		}

		localizer.Update(odoPoseDiff_LS, block.m_ScanSamples);

		pPrevBlock = &block;
	}

	// Write map
	printf("Finished processing data, writing results to files\n");
	const Localizer::Particle& bestParticle = localizer.GetMostLikelyParticle();
	MapUtils::WriteMapToFile_L2Int(bestParticle.m_Map, pOutMapFilePath, Utils::LOG2_PROB_QUANTIZATION);
}

int main(int argc, char* argv[])
{
	if (argc == 1)
	{
		printf("Provide path to file!\n");
		return 1;
	}

	// Extract arguments
	const char* pInStreamFilePath = nullptr;
	const char* pOutMapFilePath = nullptr;
	for (int i = 1; i < argc - 1; i++)
	{
		std::string arg(argv[i]);
		if (arg == "-inStreamFile")
			pInStreamFilePath = argv[i + 1];
		else if (arg == "-outMapFile")
			pOutMapFilePath = argv[i + 1];
	}

	if (!pInStreamFilePath || !pOutMapFilePath)
	{
		printf("Not all arguments are provided!\n");
		return 1;
	}

	printf("Arguments:\n");
	printf("-inStreamFile : %s\n", pInStreamFilePath);
	printf("-outMapFile   : %s\n", pOutMapFilePath);

	ProcessSensorStreamFile(pInStreamFilePath, pOutMapFilePath);

	//const char* pFilePath = argv[1];
	//TestReadStreamFile(pFilePath);
	//TestWriteMap(pFilePath); // Warning: Don't enable both! File will be overridden.
	//TestCarStatsModel();



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
