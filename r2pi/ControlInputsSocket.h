#pragma once

#include "../Common/CommonIntTypes.h"

#include "ChassisSettings.h"

#include <memory>

namespace std
{
	class thread;
}

class ChassisInterface;

class ControlInputsSocket
{
public:
	bool Start();
	void TerminateWait();
	ControlInputsSocket(ChassisInterface* pChassisInterface, const ChassisSettings::ControlSettings* pChassisCtrlSettings);
	~ControlInputsSocket();

private:
	constexpr static const char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread

	ChassisInterface* m_pChassisInterface;
	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	bool m_bRunning = false;
	const ChassisSettings::ControlSettings* m_pChassisCtrlSettings = nullptr;

	void ThreadFunction();
	void ClosePipe();
	bool ProcessMessage(const char* pMsg, uint msgSize);
};
