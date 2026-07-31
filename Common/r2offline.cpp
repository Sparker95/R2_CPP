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
#include "Exploration.h"

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
	OccupancyMap map(0.25);
	MapUtils::FillRect(map, Vector2(-1, -1), Vector2(1, 1), 100);
	MapUtils::WriteMapToFile_L2Int(map, pFilePath, 8);
}

void ProcessSensorStreamFile(const char* pInStreamFilePath, const char* pOutMapFilePath, const char* pOutLocHistoryPath,
	const Localizer::Config& localizerConfig,
	const Chassis::ChassisConfig& chassisConfig,
	const Chassis::ChassisStatsConfig& chassisStatsConfig)
{
	Chassis::ChassisModel chassisModel(chassisConfig, chassisStatsConfig);

	Localizer localizer(localizerConfig, chassisModel);

	// Read all blocks
	std::vector<ControlSensorFrame> sensorStreamBlocks;
	if (!ControlSensorFrameUtils::ReadFramesFromFile(pInStreamFilePath, sensorStreamBlocks))
	{
		printf("Error: Failed to read sensor stream file\n");
		return;
	}

	// Process data
	ControlSensorFrame& prevFrame = sensorStreamBlocks[0];
	for (uint blockId = 1; blockId < static_cast<uint>(sensorStreamBlocks.size()); blockId++)
	{
		printf("Processing block %u / %u\n", blockId, static_cast<uint>(sensorStreamBlocks.size()));
		ControlSensorFrame& frame = sensorStreamBlocks[blockId];

		// --- Copy from RobotCore::Update ---
		Chassis::ChassisInputs inputs;
		Chassis::TOdometerCounter odoDiff = Chassis::CalculateOdometerDifference(frame.m_Odometer, prevFrame.m_Odometer);
		inputs.m_MoveState = frame.m_MoveState;
		inputs.m_OdoDiff = odoDiff;
		inputs.m_Steer = frame.m_Steer;
		Localizer::Mode localizerMode = Localizer::Mode::LOC_AND_MAP;
		localizer.Update(inputs, frame.m_ScanSamples, localizerMode);
		//------------------------------------

		prevFrame = frame;

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
	OccupancyMap map;
	uint quantization;
	MapUtils::ReadMapFromFile_L2Int(mapPath, map, quantization);

	if (map.GetSize().x == 0)
		return;

	DistMap distMap = PathFinding::CalculateDistanceMap(map, 0);
	MapUtils::WriteMapToFile_L2Int(distMap, "TestDistMap.rmap", 8);

	DistMap structureMap = PathFinding::CalculateStructureMap(distMap, 0.2f, 0.2f);
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
	const char* configPath = "D:\\Projects\\Robot2\\CPP\\Config\\MainConfig.cfg";
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
		else if (p.m_Type == PathFollower::PathPoint::Type::MAIN_END)
			endPositions.push_back(p.m_Pos);
	}



	printf("Path finding has ended!\n");

	// Check path obstruciton
	bool checkObstructionResult = pathPlanner.CheckPathObstruction(map, pathPlanner.GetPath(), 1.0f, 0, nullptr);

	printf("CheckPathObstruction result: %i\n", (int)checkObstructionResult);
	printf("\n");

	// Test pose calculations
	/*
	{
		PathFinding::Node::Pose testNodePose = PathFinding::CalculateNodePose(pfConfig, 0, 0, 11.9f * PI_2X / 12.0f);
		printf("Pose calculation test\n");
	}
	*/
}

void TestExploration()
{
	OccupancyMap map;
	uint l2IntQuantization;

	const char mapPath[] = "D:\\Projects\\Robot2\\Records\\Apartment 28 06 2026\\Room2.rmap";
	//const Vector2 START_POS_W(1.6, -0.2);
	const Vector2 START_POS_W(2.72f, -1.61f);

	//const char mapPath[] = "D:\\Projects\\Robot2\\Records\\Legacy Encoder\\Office 26 Jan 2026\\Office1.rmap";
	//const Vector2 START_POS_W(0, -13);

	if (!MapUtils::ReadMapFromFile_L2Int(mapPath, map, l2IntQuantization))
		return;


	DistMap distMap = PathFinding::CalculateDistanceMap(map, 1); // Threshold = 1->unknown space is not obstacle

	MapUtils::WriteMapToFile_L2Int(distMap, "TestExplorationDistanceMap.rmap", 8);

	float DIST_MAP_THRESHOLD_W = 0.2;
	std::unordered_set<Exploration::TCellPos> expCells;
	ExpMap expMap = Exploration::CalculateExplorationMap(map, distMap, START_POS_W, DIST_MAP_THRESHOLD_W, expCells);

	std::vector<std::unordered_set<Exploration::TCellPos>> cellComponents = Exploration::FindCellGraphComponents(expCells);

	MapUtils::WriteMapToFile_L2Int(expMap, "TestExplorationMap.rmap", 8);

	std::vector<Exploration::CellComponentEval> componentsEval;
	Exploration::EvaluateSortCellComponents(START_POS_W, cellComponents, expMap, 0.8f, componentsEval);

	/////
	const auto& bestComp = componentsEval[0];
	Vector2 bestCellPos_w = expMap.PosMapToWorld(bestComp.m_NearestCell);
	printf("Best cell: %i %i, %.3f %.3f\n", bestComp.m_NearestCell.x, bestComp.m_NearestCell.y,
		bestCellPos_w.x, bestCellPos_w.y);
	float distToBestCell = Vector2::Distance(bestCellPos_w, START_POS_W);
	printf("Dist to best cell: %.3f\n", distToBestCell);

	printf("Exploration Test finished!\n");
}

void TestConfigSections()
{
	Config config("D:\\Projects\\Robot2\\CPP\\Config\\TestSections.cfg");
	printf("Finished reading config\n");
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

void TestProjectPoint()
{
	Vector2 l0(1, 1);
	Vector2 l1(3, 3);
	
	Vector2 p0 = l0; // Should not cause NAN

	float lineLen = -1;
	float projDistFrom_l0 = -1;
	Vector2 projPoint;
	MathUtils::ProjectPointOnLine(p0, l0, l1, &lineLen, &projDistFrom_l0, &projPoint);

	printf("TestProjectPoint finished\n");
}

int main(int argc, char* argv[])
{
#if TRACY_ENABLE
	char __readChar;
	printf("Connect Tracy and press any button: \n");
	std::scanf("%c", &__readChar);
#endif

	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	//TestProjectPoint();

	// WIP path finding tests
	//TestPathFinding();
	TestExploration();
	return 0;
	//TestChassisCurvatureCalculation();
	//TestConfigSections();
	//return 0;
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
	const char* pInCfgMain = nullptr;
	for (int i = 1; i < argc - 1; i++)
	{
		std::string arg(argv[i]);
		if (arg == "-inStreamFile")
			pInStreamFilePath = argv[i + 1];
		else if (arg == "-outMapFile")
			pOutMapFilePath = argv[i + 1];
		else if (arg == "-outLocHistoryFile")
			pOutLocHistoryPath = argv[i + 1];
		else if (arg == "-cfg_main")
			pInCfgMain = argv[i + 1];;
	}

	printf("Arguments:\n");
	printf("-inStreamFile      : %s\n", pInStreamFilePath);
	printf("-outMapFile        : %s\n", pOutMapFilePath);
	printf("-outLocHistoryFile : %s\n", pOutLocHistoryPath);
	printf("-cfg_main          : %s\n", pInCfgMain);

	if (!pInStreamFilePath || !pOutMapFilePath || !pOutLocHistoryPath || !pInCfgMain)
	{
		printf("Not all arguments are provided!\n");
		return 1;
	}

	// Read config files
	Config mainConfig(pInCfgMain);

	bool initDone = true;

	Chassis::ChassisConfig chassisConfig; // Using defaults
	Chassis::ChassisStatsConfig chassisStatsConfig;
	Localizer::Config localizerConfig;

	initDone = initDone && ConfigUtils::ReadChassisStatsConfig(mainConfig, chassisStatsConfig);
	initDone = initDone && ConfigUtils::ReadLocalizerConfig(mainConfig, localizerConfig);

	if (!initDone)
	{
		printf("Failed to read config files!\n");
		return 1;
	}

	ProcessSensorStreamFile(pInStreamFilePath, pOutMapFilePath, pOutLocHistoryPath, localizerConfig, chassisConfig, chassisStatsConfig);

	//_CrtDumpMemoryLeaks();

	return 0;
}