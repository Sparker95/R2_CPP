#pragma once

#include "../Common/Common.h"
#include "../Common/Sensor.h"

#include <mutex>
#include <queue>
#include <memory>
#include <vector>
#include <atomic>
#include <condition_variable>

class SensorInterface;

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
		uint m_RawSampleCount; // Count of all samples, including invalid
	};

	MainMessageLoop(SensorInterface* pSensorInterface);
	~MainMessageLoop();
	void Execute();
	void HandleOsSignal(int signalValue); // Called from interrupt!
	void PostMessage(std::unique_ptr<MessageBase>&& pMsg);

	// Getters for data, to be used by ControlPanelServer
	void GetScanSamples_mt(std::vector<Sensor::ScanSample>& outSamples, uint& outScanCounter);
	uint GetScanCounter_mt() const { return m_ScanCounter; }

private:
	// Other components
	SensorInterface* m_pSensorInterface;

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
	void OnReceivedScan_mt(const Sensor::ScanSample* pSamples, uint nValidSamples, uint nRawSamples);
};