#include "ControlPanelServer.h"
#include "ControlInputsSocket.h"
#include "SensorInterface.h"
#include "MainMessageLoop.h"
#include "ChassisInterface.h"
#include "ChassisSettings.h"

#include "../Common/Config.h"
#include "../Common/Localizer.h"
#include "../Common/LocalizerUtils.h"
#include "../Common/RobotCore.h"
#include "../Common/Chassis.h"
#include "../Common/ChassisUtils.h"
#include "../Common/ProbUtils.h"
#include "../Common/CrashDetector.h"

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

bool ReadConfigChassisInterfaceSettings(const Config& cfg, ChassisInterface::AllSettings& outSettings)
{
	ChassisInterface::AllSettings s;

	bool cfgSuccess =
		// Connection settings
		cfg.Get("SerialDevPath", s.m_ConnSettings.m_SerialDevPath) &&
		// PID settings
		cfg.Get("MotorControllerKP", s.m_PidSettings.m_KP) &&
		cfg.Get("MotorControllerKI", s.m_PidSettings.m_KI) &&
		cfg.Get("MotorControllerKD", s.m_PidSettings.m_KD) &&
		cfg.Get("MotorControllerErrorScale", s.m_PidSettings.m_ErrorScale) &&
		cfg.Get("MotorControllerErrorIntegralMax", s.m_PidSettings.m_ErrorIntegralMax) &&
		// Acceleration settings
		cfg.Get("MotorControllerAccelMove", s.m_AccelSettings.m_AccelMove) &&
		cfg.Get("MotorControllerAccelBrake", s.m_AccelSettings.m_AccelBrake) &&
		// Steering settings
		cfg.Get("SteerSpeed", s.m_SteeringSettings.m_SteeringSpeed);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read chassis interface settings from config\n");
		return false;
	}

	outSettings = s;
	return true;
}

bool ReadConfigChassisControlSettings(const Config& cfg, ChassisSettings::ControlSettings& outSettings, CrashDetector::Settings& outCrashDetectorSettings)
{
	ChassisSettings::ControlSettings s;

	bool cfgSuccess =
		cfg.Get("SteerOutputMax", s.m_SteerOutputMax) &&
		cfg.Get("SpeedSlow_Hz", s.m_SpeedSlow_Hz) &&
		cfg.Get("SpeedCruise_Hz", s.m_SpeedCruise_Hz) &&
		cfg.Get("SpeedFast_Hz", s.m_SpeedFast_Hz) &&
		cfg.Get("CrashDetector_BrakeAccel", outCrashDetectorSettings.m_BrakeAccel_freq_s2) &&
		cfg.Get("CrashDetector_ExtraTimeToCrash_s", outCrashDetectorSettings.m_ExtraTimeToCrash_s);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read chassis control settings from config\n");
		return false;
	}
	
	outSettings = s;
	return true;
}

bool ReadConfigSensorInterfaceSettings(const Config& cfg, SensorInterface::Settings& outSettings)
{
	SensorInterface::Settings s;

	bool cfgSuccess = cfg.Get("SerialDevPath", s.m_SerialDevPath) &&
		cfg.Get("MotorSpeed_Hz", s.m_MotorSpeed_Hz) &&
		cfg.Get("MotorControllerKP", s.m_MotorControllerPidSettings.m_fKP) &&
		cfg.Get("MotorControllerKI", s.m_MotorControllerPidSettings.m_fKI) &&
		cfg.Get("MotorControllerKD", s.m_MotorControllerPidSettings.m_fKD) &&
		cfg.Get("MotorControllerErrorScale", s.m_MotorControllerPidSettings.m_fErrorScale) &&
		cfg.Get("MotorControllerErrorIntegralMax", s.m_MotorControllerPidSettings.m_fErrorIntegralMax) &&
		cfg.Get("MotorControllerStartPwmValue_01", s.m_MotorControllerStartPwmValue_01) &&
		cfg.Get("MotorControllerMinPwmValue_01", s.m_MotorControllerMinPwmValue_01);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read sensor settings from config\n");
		return false;
	}

	outSettings = s;
	return true;
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
	}

	printf("Command Line Arguments:\n");
	printf("-inChassisIfConfig      : %s\n", pInChassisIfConfig);
	printf("-inChassisStatsConfig   : %s\n", pInChassisStatsConfig);
	printf("-inLocalizerConfig      : %s\n", pInLocalizerConfig);
	printf("-inSensorIfConfig       : %s\n", pInSensorIfConfig);
	printf("-inChassisControlConfig : %s\n", pInChassisControlConfig);

	if (!pInChassisIfConfig || !pInChassisStatsConfig || !pInLocalizerConfig)
	{
		printf("Not all arguments are provided!\n");
		return 1;
	}

	bool initDone = true;

	// Read config files
	ChassisInterface::AllSettings    chassisIfSettings;
	ChassisSettings::ControlSettings chassisControlSettings;
	CrashDetector::Settings crashDetectorSettings;
	SensorInterface::Settings        sensorIfSettings;
	Localizer::Config                localizerConfig;
	Chassis::ChassisStatsConfig      chassisStatsConfig;
	initDone = initDone && ReadConfigChassisInterfaceSettings(Config(pInChassisIfConfig), chassisIfSettings);
	initDone = initDone && ReadConfigChassisControlSettings(Config(pInChassisControlConfig), chassisControlSettings, crashDetectorSettings);
	initDone = initDone && ReadConfigSensorInterfaceSettings(Config(pInSensorIfConfig), sensorIfSettings);
	initDone = initDone && LocalizerUtils::ReadConfig(Config(pInLocalizerConfig), localizerConfig);
	initDone = initDone && ChassisUtils::ReadStatsConfig(Config(pInChassisStatsConfig), chassisStatsConfig);

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
	CrashDetector crashDetector(crashDetectorSettings, chassisModel);

	// Core
	RobotCore robotCore(localizer, crashDetector);

	// Sensor interface
	SensorInterface sensor(sensorIfSettings);
	if (!sensor.Start())
		initDone = false;

	// Chassis interface
	ChassisInterface chassis;
	if (!chassis.Init(chassisIfSettings))
		initDone = false;

	// Main message loop
	MainMessageLoop msgLoop(sensor, chassis, robotCore);
	g_pMsgLoop = &msgLoop;

	// Control panel server
	ControlPanelServer ctrlPanelServer(msgLoop, sensor, robotCore);
	ctrlPanelServer.Start();

	// Control inputs
	ControlInputsSocket inputsSocket(chassis, chassisControlSettings, crashDetector);
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
