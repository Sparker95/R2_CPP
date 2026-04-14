#pragma once

#include "../Common/CommonIntTypes.h"

#include "ChassisSettings.h"

#include <memory>

namespace std
{
	class thread;
}

class ChassisInterface;
class RobotCore;

namespace Chassis
{
	class ChassisModel;
}

class ControlInputsSocket
{
public:
	bool Start();
	void TerminateWait();
	ControlInputsSocket(ChassisInterface& chassisInterface, const Chassis::ChassisModel& chassisModel,
		const ChassisSettings::ControlSettings& chassisCtrlSettings,
		const RobotCore& robotCore);
	~ControlInputsSocket();

private:
	constexpr static const char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread

	ChassisInterface* m_pChassisInterface = nullptr;
	const Chassis::ChassisModel* m_pChassisModel = nullptr;
	const ChassisSettings::ControlSettings* m_pChassisCtrlSettings = nullptr;
	const RobotCore* m_pRobotCore = nullptr;

	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	bool m_bRunning = false;

	void ThreadFunction();
	void ClosePipe();
	bool ProcessMessage(const char* pMsg, uint msgSize);
};
