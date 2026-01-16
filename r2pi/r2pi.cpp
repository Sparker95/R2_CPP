#include "ControlPanelServer.h"
#include "ControlInputsSocket.h"
#include "SensorInterface.h"
#include "MainMessageLoop.h"
#include "ChassisInterface.h"
#include "ChassisSettings.h"

#include "../Common/Config.h"

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

/*
mt_ctrl_P 0.5
mt_ctrl_I 1.0
mt_ctrl_D 0.0
mt_ctrl_ErrorScale 0.001
mt_ctrl_ErrorIntegralMax 1.0
mt_ctrl_AccelMove 400.0
mt_ctrl_AccelBrake 800.0
*/

bool ReadConfigPidSettings(const Config& cfg, ChassisInterface::MotorCtrlPidSettings& outSettings)
{
	ChassisInterface::MotorCtrlPidSettings s;

	bool cfgSuccess =
		cfg.Get("mt_ctrl_P", s.m_KP) &&
		cfg.Get("mt_ctrl_I", s.m_KI) &&
		cfg.Get("mt_ctrl_D", s.m_KD) &&
		cfg.Get("mt_ctrl_ErrorScale", s.m_ErrorScale) &&
		cfg.Get("mt_ctrl_ErrorIntegralMax", s.m_ErrorIntegralMax);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read PID settings from config\n");
		return false;
	}

	// Try to preserve default values in case of failure
	outSettings = s;
	return true;
}

bool ReadConfigAccelSettings(const Config& cfg, ChassisInterface::MotorCtrlAccelSettings& outSettings)
{
	ChassisInterface::MotorCtrlAccelSettings s;

	bool cfgSuccess =
		cfg.Get("mt_ctrl_AccelMove", s.m_AccelMove) &&
		cfg.Get("mt_ctrl_AccelBrake", s.m_AccelBrake);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read acceleration settings from config\n");
		return false;
	}

	outSettings = s;
	return true;
}

bool ReadConfigChassisControlSettings(const Config& cfg, ChassisSettings::ControlSettings& outSettings)
{
	ChassisSettings::ControlSettings s;

	bool cfgSuccess =
		cfg.Get("ctrl_SteerOutputMax", s.m_SteerOutputMax) &&
		cfg.Get("ctrl_SpeedSlow_Hz", s.m_SpeedSlow_Hz) &&
		cfg.Get("ctrl_SpeedCruise_Hz", s.m_SpeedCruise_Hz) &&
		cfg.Get("ctrl_SpeedFast_Hz", s.m_SpeedFast_Hz);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read chassis control settings from config\n");
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

	bool initDone = true;

	// Read config file
	Config config;
	if (!config.ReadConfig("../../../config.cfg"))
		initDone = false;
	ChassisInterface::MotorCtrlPidSettings pidSettings;
	ChassisInterface::MotorCtrlAccelSettings accelSettings;
	ChassisSettings::ControlSettings chassisControlSettings;
	if (!ReadConfigPidSettings(config, pidSettings))
		initDone = false;
	if (!ReadConfigAccelSettings(config, accelSettings))
		initDone = false;
	if (!ReadConfigChassisControlSettings(config, chassisControlSettings))
		initDone = false;

	// Sensor interface
	SensorInterface sensor;
	if (!sensor.Start())
		initDone = false;

	// Chassis interface
	ChassisInterface chassis;
	if (!chassis.Init(pidSettings, accelSettings))
		initDone = false;

	// Main message loop
	MainMessageLoop msgLoop(sensor, chassis);
	g_pMsgLoop = &msgLoop;

	// Control panel server
	ControlPanelServer ctrlPanelServer(&msgLoop, &sensor);
	ctrlPanelServer.Start();

	// Control inputs
	ControlInputsSocket inputsSocket(&chassis, &chassisControlSettings);
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
