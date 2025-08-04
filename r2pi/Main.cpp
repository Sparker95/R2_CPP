#include "ControlPanelServer.h"
#include "SensorInterface.h"
#include "MainMessageLoop.h"

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

	bool initDone = true;

	// Stats server
	ControlPanelServer statsServer;
	statsServer.Start();

	// Sensor interface
	SensorInterface sensor;
	if (!sensor.Start())
		initDone = false;

	// Main message loop
	MainMessageLoop msgLoop(&sensor);
	g_pMsgLoop = &msgLoop;

	if (initDone)
	{
		msgLoop.Execute();
	}

	if (!initDone)
		fprintf(stderr, "Error: Failed to initialize\n");
	printf("Terminating other threads\n");
	statsServer.TerminateWait();
	sensor.TerminateWait();
	printf("Program has been terminated\n");
	g_pMsgLoop = nullptr;
	return 0;
}
