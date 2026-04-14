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
	const char* pInCfgMain = nullptr;
	const char* pInCfgPlatform = nullptr;
	for (int i = 1; i < argc - 1; i++)
	{
		std::string arg(argv[i]);
		if (arg == "-cfg_main")
			pInCfgMain = argv[i + 1];
		else if (arg == "-cfg_platform")
			pInCfgPlatform = argv[i + 1];
	}

	printf("Command Line Arguments:\n");
	printf("-cfg_main     : %s\n", pInCfgMain);
	printf("-cfg_platform : %s\n", pInCfgPlatform);

	if (!pInCfgMain || !pInCfgPlatform)
	{
		printf("Not all arguments are provided!\n");
		return 1;
	}

	bool initDone = true;

	// Read config files
	Config mainConfig(pInCfgMain);
	Config platformConfig(pInCfgPlatform);

	ChassisInterface::AllSettings    chassisIfSettings;
	ChassisSettings::ControlSettings chassisControlSettings;
	CrashDetector::Config            crashDetectorConfig;
	SensorInterface::Settings        sensorIfSettings;
	Localizer::Config                localizerConfig;
	Chassis::ChassisStatsConfig      chassisStatsConfig;
	PathFinding::Config              pathFindingConfig;
	PathFollowerConfig               pathFollowerConfig;
	initDone = initDone && PlatformConfigUtils::ReadChassisInterfaceSettingsConfig(platformConfig, chassisIfSettings);
	initDone = initDone && PlatformConfigUtils::ReadChassisControlSettingsConfig(platformConfig, chassisControlSettings);
	initDone = initDone && PlatformConfigUtils::ReadSensorInterfaceSettingsConfig(platformConfig, sensorIfSettings);
	initDone = initDone && ConfigUtils::ReadCrashDetectorConfig(mainConfig, crashDetectorConfig);
	initDone = initDone && ConfigUtils::ReadLocalizerConfig(mainConfig, localizerConfig);
	initDone = initDone && ConfigUtils::ReadChassisStatsConfig(mainConfig, chassisStatsConfig);
	initDone = initDone && ConfigUtils::ReadPathFindingConfig(mainConfig, pathFindingConfig);
	initDone = initDone && ConfigUtils::ReadPathFollowerConfig(mainConfig, pathFollowerConfig);

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
	ControlInputsSocket inputsSocket(chassis, chassisModel, chassisControlSettings, robotCore);
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
