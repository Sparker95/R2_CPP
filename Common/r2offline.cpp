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

#include "MathUtils.h"
#include "Config.h"
#include "ConfigUtils.h"
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
#include "CrashDetector.h"
#include "PathFinding.h"
#include "PathPlanner.h"
#include "PathFollower.h"

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

void ProcessSensorStreamFile(const char* pInStreamFilePath, const char* pOutMapFilePath, const char* pOutLocHistoryPath,
	const Localizer::Config& localizerConfig, const Chassis::ChassisStatsConfig& chassisStatsConfig)
{
	Chassis::ChassisConfig chassisConfig;
	Chassis::ChassisModel chassisModel(chassisConfig, chassisStatsConfig);

	CrashDetector::Config crashDetectorSettings;
	CrashDetector crashDetector(crashDetectorSettings, chassisModel);

	PathFollowerConfig followerConfig;
	PathFinding::Config pfConfig;
	PathPlanner pathPlanner(pfConfig, followerConfig);

	Localizer localizer(localizerConfig, chassisModel);

	RobotCore core(localizer, crashDetector, pathPlanner);
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

void TestPathFinding()
{
	const char* mapPath = "D:\\Projects\\Robot2\\Records\\Legacy Encoder\\Office 26 Jan 2026\\Office1.rmap";
	//const char* mapPath = "/home/pi/projects/r2offline_pi/Office1.rmap";
	Map map;
	uint quantization;
	MapUtils::ReadMapFromFile_L2Int(mapPath, map, quantization);

	if (map.GetSize().x == 0)
		return;

	Map distMap = PathFinding::CalculateDistanceMap(map, 0);
	MapUtils::WriteMapToFile_L2Int(distMap, "TestDistMap.rmap", 8);

	Map structureMap = PathFinding::CalculateStructureMap(distMap, 0.2f, 0.2f);
	MapUtils::WriteMapToFile_L2Int(structureMap, "TestStructureMap.rmap", 8);

	// Test rect rasterizer
	std::vector<LineRasterizer::HSpan> rectSpans;
	LineRasterizer::RasterizeRect({ 5.6, 7.8 }, { 3.3, 4.4 }, { std::cos(0.3f), std::sin(0.3f) }, rectSpans);
	//LineRasterizer::RasterizeRect({ 0, 0 }, { 3.1, 3.1 }, { std::cos(0.0f), std::sin(0.0f) }, rectSpans);
	printf("Rectangle has been rasterized!\n");

	// Test line rasterizer
	std::vector<Vector2Base<int>> lineCells;
	LineRasterizer::RasterizeLine({ 0, 0 }, { 4.3, 7.1 }, lineCells);
	printf("Line rasterization has ended!\n");

	// Test path finding
	PathFinding::Config pfConfig;
	const char* configPath = "D:\\Projects\\Robot2\\CPP\\Config\\PathFinding.cfg";
	//const char* configPath = "/home/pi/projects/r2pi/PathFinding.cfg"; // Linux
	if (!ConfigUtils::ReadPathFindingConfig(Config(configPath), pfConfig))
	{
		printf("Failed to read Path Finding config! We can't find path without config!\n");
		return;
	}
	PathFollowerConfig followerConfig;
	PathPlanner pathPlanner(pfConfig, followerConfig);

	Pose startPose = { 4.4f, -1.0f, 0 };
	Vector2 destPos(-1, -4.5f); // The next room
	float destRadius = 0.3f;

	//PathFinding::PathFindingResult pfResult;
	//PathFinding::FindPath(pfConfig, map, startNodePose, destPos, destRadius, pfResult);

	pathPlanner.PlanPath(map, startPose, destPos, destRadius);

	std::vector<Vector2> pathPositions;
	std::vector<Vector2> endPositions;
	std::vector<Vector2> extPositions;
	for (const auto& p : pathPlanner.GetPath())
	{
		pathPositions.push_back(p.m_Pos);
		if (p.m_Type == PathFollower::PathPoint::Type::EXTENSION)
			extPositions.push_back(p.m_Pos);
		else if (p.m_Type == PathFollower::PathPoint::Type::END)
			endPositions.push_back(p.m_Pos);
	}



	printf("Path finding has ended!\n");


	// Test pose calculations
	/*
	{
		PathFinding::Node::Pose testNodePose = PathFinding::CalculateNodePose(pfConfig, 0, 0, 11.9f * PI_2X / 12.0f);
		printf("Pose calculation test\n");
	}
	*/
}

void TestLineX()
{
	Vector2 intersections[2];
	uint nIntersections = MathUtils::CircleXLine({ -5, 5 }, 5.0f,
		{ -10,0 }, { 10,7 },
		intersections);

	printf("Circle X Line intersection test\n");
}

void TestChassisCurvatureCalculation()
{
	Chassis::ChassisConfig chassisConfig;
	Chassis::ChassisStatsConfig statsConfig;
	Chassis::ChassisModel chassisModel(chassisConfig, statsConfig);

	float curvature = -0.05;

	float steeringInput = chassisModel.CalculateSteeringInput(curvature);
	printf("Calculated steering input. %.3f -> %.3f\n", curvature, steeringInput);
}

int main(int argc, char* argv[])
{
#if TRACY_ENABLE
	char __readChar;
	printf("Connect Tracy and press any button: \n");
	std::scanf("%c", &__readChar);
#endif

	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	// WIP path finding tests
	TestPathFinding();
	//TestChassisCurvatureCalculation();
	return 0;
	///////////////////////////

	if (argc == 1)
	{
		printf("Provide path to file!\n");
		return 1;
	}

	// Extract arguments
	const char* pInStreamFilePath = nullptr;
	const char* pOutMapFilePath = nullptr;
	const char* pOutLocHistoryPath = nullptr;
	const char* pInLocalizerConfig = nullptr;
	const char* pInChassisConfig = nullptr;
	for (int i = 1; i < argc - 1; i++)
	{
		std::string arg(argv[i]);
		if (arg == "-inStreamFile")
			pInStreamFilePath = argv[i + 1];
		else if (arg == "-outMapFile")
			pOutMapFilePath = argv[i + 1];
		else if (arg == "-outLocHistoryFile")
			pOutLocHistoryPath = argv[i + 1];
		else if (arg == "-inChassisConfig")
			pInChassisConfig = argv[i + 1];
		else if (arg == "-inLocalizerConfig")
			pInLocalizerConfig = argv[i + 1];
	}

	printf("Arguments:\n");
	printf("-inStreamFile      : %s\n", pInStreamFilePath);
	printf("-outMapFile        : %s\n", pOutMapFilePath);
	printf("-outLocHistoryFile : %s\n", pOutLocHistoryPath);
	printf("-inChassisConfig   : %s\n", pInChassisConfig);
	printf("-inLocalizerConfig : %s\n", pInLocalizerConfig);

	if (!pInStreamFilePath || !pOutMapFilePath || !pOutLocHistoryPath || !pInChassisConfig || !pInLocalizerConfig)
	{
		printf("Not all arguments are provided!\n");
		return 1;
	}

	// Read config files
	Chassis::ChassisStatsConfig chassisStatsConfig;
	Localizer::Config localizerConfig;

	if (!ConfigUtils::ReadChassisStatsConfig(Config(pInChassisConfig), chassisStatsConfig) ||
		!ConfigUtils::ReadLocalizerConfig(Config(pInLocalizerConfig), localizerConfig))
	{
		printf("Failed to read config files!\n");
		return 1;
	}

	ProcessSensorStreamFile(pInStreamFilePath, pOutMapFilePath, pOutLocHistoryPath, localizerConfig, chassisStatsConfig);

	//TestBresenhamLine();
	//const char* pFilePath = argv[1];
	//TestReadStreamFile(pFilePath);
	//TestWriteMap(pFilePath); // Warning: Don't enable both! File will be overridden.
	//TestCarStatsModel();

	//_CrtDumpMemoryLeaks();

	return 0;
}