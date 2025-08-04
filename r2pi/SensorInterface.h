#pragma once

#include "../Common/Common.h"
#include "../Common/Sensor.h"

#include <memory>
#include <functional>
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

		void Convert(float& outAngle_rad, float& outDistance_m);
	};

public:
	bool Start();
	void TerminateWait();
	SensorInterface();
	~SensorInterface();

	// !!! Invoked in thread of SensorInterface!
	// Here we must copy scan data and return.
	template<typename F>
	void BindOnReceivedScan(F f) { m_OnReceivedScan = f; }

private:
	constexpr static char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread
	
	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	int m_SerialPortFd = -1;
	bool m_bRunning = false;

	// Processing of scan data packets, used by ReadScanDataStream
	std::unique_ptr<RawScanSample[]> m_pRawScanSampleBuffer;
	std::unique_ptr<Sensor::ScanSample[]> m_pOutScanSamples;

	uint m_ReceivedScanCount = 0; // Total number of scans received
	uint m_PacketByteId = 0;
	uint m_ScanSampleId = 0;
	uchar m_ScanPacketBuffer[5]; // Packet is read byte by byte into this buffer
	bool m_WaitingFirstScanStart = true;

	// Handling of events
	// !!! Invoked in thread of SensorInterface!
	std::function<void(const Sensor::ScanSample*, uint, uint)> m_OnReceivedScan;

	void ThreadFunction();
	void ClosePipe();
	void CloseSerialPort();

	void ReadScanDataStream(const uchar* pData, uint nBytes);
	void ProcessScanDataPacket();
	void ProcessScanResult(); // Convert raw data to floats, invoke events

	bool InitSensor();
};