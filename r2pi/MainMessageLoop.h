#pragma once

#include "../Common/Common.h"
#include "../Common/Sensor.h"

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

	MainMessageLoop(SensorInterface* pSensorInterface, ChassisInterface* pChassisInterface);
	~MainMessageLoop();
	void Init(ControlPanelServer* pCtrlPanelServer);
	void Execute();
	void HandleOsSignal(int signalValue); // Called from interrupt!
	void PostMessage(std::unique_ptr<MessageBase>&& pMsg);

private:
	// Other components
	SensorInterface* m_pSensorInterface;
	ChassisInterface* m_pChassisInterface;
	ControlPanelServer* m_pCtrlPanelServer = nullptr;

	// Timestamp
	std::chrono::high_resolution_clock::time_point m_TimeStart = {};

	// Queue
	std::mutex m_QueueMutex;
	std::condition_variable m_QueueCondVariable;
	std::queue<std::unique_ptr<MessageBase>> m_MessageQueue;
	std::atomic<int> m_SignalValue = 0;

	// Processing of messages
	std::mutex m_StateMutex; // Locked when we modify our state

	// Data from sensors
	std::vector<Sensor::ScanSample> m_ScanSamples;
	std::atomic<uint> m_ScanCounter = 0;

	void ProcessMessage(MessageBase& msgBase);

	// Called from SensorInterface thread!
	void OnReceivedScan_mt(const std::vector<Sensor::ScanSample>&,
		const std::vector<Sensor::ScanSampleInt>&);
};