#pragma once

#include "../Common/CommonIntTypes.h"
#include "../Common/Sensor.h"

#include "PidController.h"

#include "Pigpio.h"

#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <stdint.h>
#include <mutex>
#include <chrono>
#include <atomic>

namespace std
{
	class thread;
}

class SensorInterface
{
public:
	using TTimestamp = std::chrono::high_resolution_clock::time_point;

	struct RawScanSample
	{
		ushort m_RawAngle;
		ushort m_RawDistance;
		uchar  m_Quality;
	};

public:
	struct Settings
	{
		std::string m_SerialDevPath;
		float m_MotorSpeed_Hz = 0;
		PidController::Settings m_MotorControllerPidSettings;
		float m_MotorControllerStartPwmValue_01 = 1.0; // Initial value used to start the motor
		float m_MotorControllerMinPwmValue_01 = 0;
	};

	bool Start();
	void TerminateWait();
	SensorInterface(const Settings& settings);
	~SensorInterface();

	// !!! Invoked in thread of SensorInterface!
	// Here we must copy scan data and return.
	template<typename F>
	void BindOnReceivedScan(F f) { m_OnReceivedScan.push_back(f); }

	// Motor control
	bool SetMotorState_mt(bool enable);
	bool GetMotorState_mt();

private:
	constexpr static char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread

	// const std::vector<Sensor::ScanSample>& scanSamples,	const std::vector<Sensor::ScanSampleInt>& scanSamplesInt
	// TTimestamp timeEnd, float scanPeriod_s
	using TOnReceivedScan = std::function<void(const std::vector<Sensor::ScanSample>&,
												const std::vector<Sensor::ScanSampleInt>&,
												TTimestamp,
												float)>;
	Settings m_Settings;
	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	int m_SerialPortFd = -1;
	bool m_bRunning = false;

	// Processing of scan data packets, used by ReadScanDataStream
	std::vector<RawScanSample> m_RawScanSampleBuffer;
	std::vector<Sensor::ScanSample> m_OutScanSamples;
	std::vector<Sensor::ScanSampleInt> m_OutIntScanSamples;

	TTimestamp m_TimeLastScanStart = {};
	uint m_ReceivedScanCount = 0; // Total number of scans received
	uint m_PacketByteId = 0;
	uchar m_ScanPacketBuffer[5]; // Packet is read byte by byte into this buffer
	bool m_WaitingFirstScanStart = true;

	// Handling of events
	// !!! Invoked in thread of SensorInterface!
	std::vector<TOnReceivedScan> m_OnReceivedScan;

	// Motor control, GPIO
	Pigpio m_Gpio;
	std::mutex m_GpioMutex;
	PidController m_MotorPidController;
	std::atomic_bool m_MotorState = false; // Atomic since it's read from another thread

	void ThreadFunction();
	void ClosePipe();
	void CloseSerialPort();

	void ReadScanDataStream(const uchar* pData, uint nBytes);
	void ProcessScanDataPacket();
	void ProcessScanResult(TTimestamp timeNow, float scanPeriod_s); // Convert raw data to floats, invoke events
	void UpdateMotorPidController(float dt_s);

	bool InitSensor();
};