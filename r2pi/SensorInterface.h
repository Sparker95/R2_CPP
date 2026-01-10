#pragma once

#include "../Common/CommonIntTypes.h"
#include "../Common/Sensor.h"

#include "Pigpio.h"

#include <memory>
#include <functional>
#include <vector>
#include <stdint.h>

namespace std
{
	class thread;
}

class SensorInterface
{
public:
	struct RawScanSample
	{
		ushort m_RawAngle;
		ushort m_RawDistance;
		uchar  m_Quality;
	};

public:
	bool Start();
	void TerminateWait();
	SensorInterface();
	~SensorInterface();

	// !!! Invoked in thread of SensorInterface!
	// Here we must copy scan data and return.
	template<typename F>
	void BindOnReceivedScan(F f) { m_OnReceivedScan.push_back(f); }

private:
	constexpr static char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread

	// const std::vector<Sensor::ScanSample>& scanSamples,	const std::vector<Sensor::ScanSampleInt>& scanSamplesInt
	using TOnReceivedScan = std::function<void(const std::vector<Sensor::ScanSample>&,
												const std::vector<Sensor::ScanSampleInt>&)>;

	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	int m_SerialPortFd = -1;
	bool m_bRunning = false;

	// Processing of scan data packets, used by ReadScanDataStream
	std::vector<RawScanSample> m_RawScanSampleBuffer;
	std::vector<Sensor::ScanSample> m_OutScanSamples;
	std::vector<Sensor::ScanSampleInt> m_OutIntScanSamples;

	uint m_ReceivedScanCount = 0; // Total number of scans received
	uint m_PacketByteId = 0;
	uchar m_ScanPacketBuffer[5]; // Packet is read byte by byte into this buffer
	bool m_WaitingFirstScanStart = true;

	// Handling of events
	// !!! Invoked in thread of SensorInterface!
	std::vector<TOnReceivedScan> m_OnReceivedScan;

	// GPIO
	Pigpio m_Gpio;

	void ThreadFunction();
	void ClosePipe();
	void CloseSerialPort();
	bool StartMotor();
	bool StopMotor();

	void ReadScanDataStream(const uchar* pData, uint nBytes);
	void ProcessScanDataPacket();
	void ProcessScanResult(); // Convert raw data to floats, invoke events

	bool InitSensor();
};