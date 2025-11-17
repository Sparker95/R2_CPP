#include "ControlPanelServer.h"
#include "ControlInputsSocket.h"
#include "SensorInterface.h"
#include "MainMessageLoop.h"
#include "ChassisInterface.h"
#include "Config.h"

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

	const char* pP = cfg.FindValue("mt_ctrl_P");
	const char* pI = cfg.FindValue("mt_ctrl_I");
	const char* pD = cfg.FindValue("mt_ctrl_D");
	const char* pErrorScale = cfg.FindValue("mt_ctrl_ErrorScale");
	const char* pErrorIntegralMax = cfg.FindValue("mt_ctrl_ErrorIntegralMax");

	if (!pP || !pI || !pD || !pD || !pErrorScale || !pErrorIntegralMax)
	{
		fprintf(stderr, "Error: Failed to read PID settings from config\n");
		return false;
	}

	s.m_fKP = static_cast<float>(std::atof(pP));
	s.m_fKI = static_cast<float>(std::atof(pI));
	s.m_fKD = static_cast<float>(std::atof(pD));
	s.m_fErrorScale = static_cast<float>(std::atof(pErrorScale));
	s.m_fErrorIntegralMax = static_cast<float>(std::atof(pErrorIntegralMax));

	// Try to preserve default values in case of failure
	outSettings = s;
	return true;
}

bool ReadConfigAccelSettings(const Config& cfg, ChassisInterface::MotorCtrlAccelSettings& outSettings)
{
	ChassisInterface::MotorCtrlAccelSettings s;

	const char* pAccelMove = cfg.FindValue("mt_ctrl_AccelMove");
	const char* pAccelBrake = cfg.FindValue("mt_ctrl_AccelBrake");

	if (!pAccelMove || !pAccelBrake)
	{
		fprintf(stderr, "Error: Failed to read acceleration settings from config\n");
		return false;
	}

	s.m_fAccelMove = static_cast<float>(std::atof(pAccelMove));
	s.m_fAccelBrake = static_cast<float>(std::atof(pAccelBrake));

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
	if (!ReadConfigPidSettings(config, pidSettings))
		initDone = false;
	if (!ReadConfigAccelSettings(config, accelSettings))
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
	MainMessageLoop msgLoop(&sensor);
	g_pMsgLoop = &msgLoop;

	// Control panel server
	ControlPanelServer ctrlPanelServer(&msgLoop);
	ctrlPanelServer.Start();

	// Control inputs
	ControlInputsSocket inputsSocket(&chassis);
	inputsSocket.Start();

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
