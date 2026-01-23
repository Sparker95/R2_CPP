#pragma once

#include "../Common/Common.h"
#include "../Common/Sensor.h"
#include "../Common/ControlPanelProtocol.h"
#include "../Common/ControlSensorFrame.h"

#include <mutex>
#include <queue>
#include <memory>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <chrono>

class SensorInterface;
class ChassisInterface;
class ControlPanelServer;
class RobotCore;

class MainMessageLoop
{
public:
	struct MessageBase
	{
		virtual ~MessageBase() = default;
	};

	struct MessageNewSensorScanData : MessageBase
	{
		std::vector<Sensor::ScanSample> m_Scan;
		std::vector<Sensor::ScanSampleInt> m_ScanInt;
	};

	// For all trivial messaages from control panel.
	// m_Data contains the data of the message.
	struct MessageControlPanel : MessageBase
	{
		constexpr static unsigned int MAX_PAYLOAD_SIZE = 32;

		ControlPanelProtocol::MessageType m_Type = {};
		uint8_t _padding0;
		uint8_t _padding1;
		uint8_t _padding2;
		char m_Payload[MAX_PAYLOAD_SIZE] = {};

		template<class T>
		T& CastPayload() { return *reinterpret_cast<T*>(*m_Payload); }
	};

	MainMessageLoop(SensorInterface& sensorInterface, ChassisInterface& cHassisInterface,
		RobotCore& robotCore);
	~MainMessageLoop();
	void Init(ControlPanelServer* pCtrlPanelServer);
	void Execute();
	void HandleOsSignal(int signalValue); // Called from interrupt!
	void PostMessage(std::unique_ptr<MessageBase>&& pMsg);

private:
	// Other components
	SensorInterface* m_pSensorInterface = nullptr;
	ChassisInterface* m_pChassisInterface = nullptr;
	ControlPanelServer* m_pCtrlPanelServer = nullptr;
	RobotCore* m_pRobotCore = nullptr;

	// Timestamp
	std::chrono::high_resolution_clock::time_point m_TimeStart = {};

	// Queue
	std::mutex m_QueueMutex;
	std::condition_variable m_QueueCondVariable;
	std::queue<std::unique_ptr<MessageBase>> m_MessageQueue;
	std::atomic<int> m_SignalValue = 0;

	// Processing of messages
	std::mutex m_StateMutex; // Locked when we modify state of objects 'attached' to this thread, like RobotCore

	// Statistics
	uint m_SensorScanCounter = 0;

	// Related to RobotCore update
	ControlSensorFrame m_CtrlSensorFrame; // To advoid array reallocations

	void ProcessMessage(MessageBase& msgBase);

	// Called from SensorInterface thread!
	void OnReceivedScan_mt(const std::vector<Sensor::ScanSample>&,
		const std::vector<Sensor::ScanSampleInt>&);
};