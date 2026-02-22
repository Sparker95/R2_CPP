// Heap memory leak detection
// https://learn.microsoft.com/en-us/cpp/c-runtime-library/find-memory-leaks-using-the-crt-library?view=msvc-170
//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>

#include <cstdio>
#include <string>
#include <fstream>

// Tracy
#ifdef TRACY_ENABLE
#include "Tracy/tracy/Tracy.hpp"
#else
#include "TracyPlug.h"
#endif

#include "Chassis.h"
#include "Localizer.h"
#include "LocalizerUtils.h"
#include "CarStatsModel.h"
#include "CommonIntTypes.h"
#include "Map.h"
#include "ControlSensorFrame.h"
#include "ControlSensorFrameUtils.h"
#include "MapUtils.h"
#include "MathUtils.h"
#include "Utils.h"
#include "ProbUtils.h"
#include "LineRasterizer.h"
#include "RobotCore.h"

void TestReadStreamFile(const char* pFilePath)
{
	printf("Reading stream file: %s\n", pFilePath);

	std::vector<ControlSensorFrame> blocks;
	bool readSuccess = ControlSensorFrameUtils::ReadFramesFromFile(pFilePath, blocks);

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

	Chassis::ChassisModel chassisModel(chassisConfig, chassisStatsConfig);

	Localizer::Config localizerConfig;
	const float _mapRes = 0.0625;
	localizerConfig.m_SensorPdfWidth = 2.0f * _mapRes;
	localizerConfig.m_SensorPdfFloor = 0.6f;
	localizerConfig.m_SensorScanRange = 12.0f;
	localizerConfig.m_MapRes_w = _mapRes;
	localizerConfig.m_StartPose = Pose(0, 0, 0);
	localizerConfig.m_nParticles = 256; // 128
	localizerConfig.m_L2IntOccupiedCell = Utils::L2FloatToL2Int(3.0f);
	localizerConfig.m_L2IntFreeCell = Utils::L2FloatToL2Int(0.5f);
	localizerConfig.m_ScanDecimation = 1;

	Localizer localizer(localizerConfig, chassisModel);

	RobotCore core(localizer);
	core.SetUpdateLocalizer(true);

	// Read all blocks
	std::vector<ControlSensorFrame> sensorStreamBlocks;
	if (!ControlSensorFrameUtils::ReadFramesFromFile(pInStreamFilePath, sensorStreamBlocks))
	{
		printf("Error: Failed to read sensor stream file\n");
		return;
	}

	// Process data
	for (uint blockId = 0; blockId < static_cast<uint>(sensorStreamBlocks.size()); blockId++)
	{
		printf("Processing block %u / %u\n", blockId, static_cast<uint>(sensorStreamBlocks.size()));
		ControlSensorFrame& frame = sensorStreamBlocks[blockId];

		core.Update(frame);

		// Tracy
		FrameMark;
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

#if TRACY_ENABLE
	char __readChar;
	printf("Connect Tracy and press any button: \n");
	std::scanf("%c", &__readChar);
#endif

	ProcessSensorStreamFile(pInStreamFilePath, pOutMapFilePath, pOutLocHistoryPath);

	//TestBresenhamLine();
	//const char* pFilePath = argv[1];
	//TestReadStreamFile(pFilePath);
	//TestWriteMap(pFilePath); // Warning: Don't enable both! File will be overridden.
	//TestCarStatsModel();

	//_CrtDumpMemoryLeaks();

	return 0;
}