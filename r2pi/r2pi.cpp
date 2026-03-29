#include "ControlPanelServer.h"
#include "ControlInputsSocket.h"
#include "SensorInterface.h"
#include "MainMessageLoop.h"
#include "ChassisInterface.h"
#include "ChassisSettings.h"

#include "../Common/Config.h"
#include "../Common/ConfigUtils.h"
#include "../Common/Localizer.h"
#include "../Common/LocalizerUtils.h"
#include "../Common/RobotCore.h"
#include "../Common/Chassis.h"
#include "../Common/ProbUtils.h"
#include "../Common/CrashDetector.h"
#include "../Common/PathFinding.h"
#include "../Common/PathPlanner.h"
#include "../Common/PathFollower.h"

#include "PlatformConfigUtils.h"

#include <cstring>
#include <thread>
#include <atomic>
#include <stdio.h>
#include <signal.h>

std::atomic<MainMessageLoop*> g_pMsgLoop = nullptr;

void SignalHandler(int s)
{
	auto* pMsgLoop = g_pMsgLoop.load();
	if (pMsgLoop)
		pMsgLoop->HandleOsSignal(s);
}

int main(int argc, char** argv)
{
	printf("Program started. PID: %i\n", getpid());

	// Linux signal handling
	{
		struct sigaction sa;
		std::memset(&sa, 0, sizeof(sa));

		sa.sa_handler = &SignalHandler;
		if (sigaction(SIGINT, &sa, nullptr) == -1)
			printf("sigaction SIGINT error\n");

		sa.sa_handler = &SignalHandler;
		if (sigaction(SIGTERM, &sa, nullptr) == -1)
			printf("sigaction SIGTERM error\n");
	}

	// Extract command line arguments
	const char* pInLocalizerConfig = nullptr;
	const char* pInChassisStatsConfig = nullptr;
	const char* pInChassisIfConfig = nullptr;
	const char* pInSensorIfConfig = nullptr;
	const char* pInChassisControlConfig = nullptr;
	const char* pInCrashDetectorConfig = nullptr;
	const char* pInPathFindingConfig = nullptr;
	for (int i = 1; i < argc - 1; i++)
	{
		std::string arg(argv[i]);
		if (arg == "-inChassisIfConfig")
			pInChassisIfConfig = argv[i + 1];
		else if (arg == "-inChassisStatsConfig")
			pInChassisStatsConfig = argv[i + 1];
		else if (arg == "-inLocalizerConfig")
			pInLocalizerConfig = argv[i + 1];
		else if (arg == "-inSensorIfConfig")
			pInSensorIfConfig = argv[i + 1];
		else if (arg == "-inChassisControlConfig")
			pInChassisControlConfig = argv[i + 1];
		else if (arg == "-inCrashDetectorConfig")
			pInCrashDetectorConfig = argv[i + 1];
		else if (arg == "-inPathFindingConfig")
			pInPathFindingConfig = argv[i + 1];
	}

	printf("Command Line Arguments:\n");
	printf("-inChassisIfConfig      : %s\n", pInChassisIfConfig);
	printf("-inChassisStatsConfig   : %s\n", pInChassisStatsConfig);
	printf("-inLocalizerConfig      : %s\n", pInLocalizerConfig);
	printf("-inSensorIfConfig       : %s\n", pInSensorIfConfig);
	printf("-inChassisControlConfig : %s\n", pInChassisControlConfig);
	printf("-inCrashDetectorConfig  : %s\n", pInCrashDetectorConfig);
	printf("-inPathFindingConfig    : %s\n", pInPathFindingConfig);

	if (!pInChassisIfConfig || !pInChassisStatsConfig || !pInLocalizerConfig || !pInSensorIfConfig ||
		!pInChassisControlConfig || !pInCrashDetectorConfig || !pInPathFindingConfig)
	{
		printf("Not all arguments are provided!\n");
		return 1;
	}

	bool initDone = true;

	// Read config files
	ChassisInterface::AllSettings    chassisIfSettings;
	ChassisSettings::ControlSettings chassisControlSettings;
	CrashDetector::Config crashDetectorConfig;
	SensorInterface::Settings        sensorIfSettings;
	Localizer::Config                localizerConfig;
	Chassis::ChassisStatsConfig      chassisStatsConfig;
	PathFinding::Config              pathFindingConfig;
	PathFollowerConfig               pathFollowerConfig;
	initDone = initDone && PlatformConfigUtils::ReadChassisInterfaceSettingsConfig(Config(pInChassisIfConfig), chassisIfSettings);
	initDone = initDone && PlatformConfigUtils::ReadChassisControlSettingsConfig(Config(pInChassisControlConfig), chassisControlSettings);
	initDone = initDone && PlatformConfigUtils::ReadSensorInterfaceSettingsConfig(Config(pInSensorIfConfig), sensorIfSettings);
	initDone = initDone && ConfigUtils::ReadCrashDetectorConfig(Config(pInCrashDetectorConfig), crashDetectorConfig);
	initDone = initDone && ConfigUtils::ReadLocalizerConfig(Config(pInLocalizerConfig), localizerConfig);
	initDone = initDone && ConfigUtils::ReadChassisStatsConfig(Config(pInChassisStatsConfig), chassisStatsConfig);
	initDone = initDone && ConfigUtils::ReadPathFindingConfig(Config(pInPathFindingConfig), pathFindingConfig);

	if (!initDone)
	{
		printf("Failed to read config files!\n");
		return 1;
	}


	// The RobotCore and related parts
	// Chassis
	Chassis::ChassisConfig chassisConfig; // Using defaults
	Chassis::ChassisModel chassisModel(chassisConfig, chassisStatsConfig);

	// Localizer
	Localizer localizer(localizerConfig, chassisModel);

	// Crash detector
	CrashDetector crashDetector(crashDetectorConfig, chassisModel);

	// Path follower
	PathFollower pathFollower(pathFollowerConfig);

	// Path planner
	PathPlanner pathPlanner(pathFindingConfig, pathFollowerConfig);

	// Core
	RobotCore robotCore(chassisModel, localizer, crashDetector, pathPlanner, pathFollower);

	// Sensor interface
	SensorInterface sensor(sensorIfSettings);
	if (!sensor.Start())
		initDone = false;

	// Chassis interface
	ChassisInterface chassis;
	if (!chassis.Init(chassisIfSettings))
		initDone = false;

	// Main message loop
	MainMessageLoop msgLoop(sensor, chassis, chassisControlSettings, robotCore);
	g_pMsgLoop = &msgLoop;

	// Control panel server
	ControlPanelServer ctrlPanelServer(msgLoop, sensor, robotCore);
	ctrlPanelServer.Start();

	// Control inputs
	ControlInputsSocket inputsSocket(chassis, chassisControlSettings, robotCore);
	inputsSocket.Start();

	// Finish message loop init
	msgLoop.Init(&ctrlPanelServer);

	if (initDone)
	{
		msgLoop.Execute();
	}

	if (!initDone)
		fprintf(stderr, "Error: Failed to initialize\n");
	printf("Terminating other threads\n");
	ctrlPanelServer.TerminateWait();
	inputsSocket.TerminateWait();
	sensor.TerminateWait();
	chassis.Terminate();
	printf("Program has been terminated\n");
	g_pMsgLoop = nullptr;
	return 0;
}
