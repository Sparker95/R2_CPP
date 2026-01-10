#pragma once

#include <memory>
#include <vector>

#include "../Common/Common.h"
#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Chassis.h"

namespace std
{
	class thread;
}

class MainMessageLoop;
class SensorInterface;

class ControlPanelServer
{
public:
	bool Start();
	void TerminateWait();
	ControlPanelServer(MainMessageLoop* pMainMsgLoop, SensorInterface* pSensorInterface);
	~ControlPanelServer();

	bool GetConnected() const;
	bool GetStreamSensorData() const;

	// Called from SensorInterface thread!
	void SendSensorData_mt(uint timestamp_ms, const std::vector<Sensor::ScanSampleInt>& scan, Chassis::TOdometerCounter odometer,
		Chassis::MoveState moveState, float steer);

private:
	constexpr static const char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread

	// Pipe message processing
	struct PipeMsgBase
	{
		uint m_Priority = 0;
		virtual ~PipeMsgBase() {};
	};
	struct PipeMsgTerminate : PipeMsgBase
	{
		PipeMsgTerminate() { m_Priority = 100; }
	};
	struct PipeMsgSensorData : PipeMsgBase
	{
		std::vector<Sensor::ScanSampleInt> m_ScanSamples;
		float m_Steer = 0;
		Chassis::TOdometerCounter m_Odometer = 0;
		Chassis::MoveState m_MoveState = {};
		uint m_Timestamp_ms = 0;

		PipeMsgSensorData() { m_Priority = 10; }
	};


	// Member variables:

	MainMessageLoop* m_pMainMsgLoop = nullptr;
	SensorInterface* m_pSensorInterface = nullptr;

	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	std::vector<PipeMsgBase*> m_PipeRxQueue; // Messages are taken from pipe into this array
	int m_ClientSockFd = -1;
	bool m_bRunning = false;
	bool m_bStreamSensorData = false;

	void ThreadFunction();
	void ClosePipe();

	void OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize);
	void OnProtocolError(uint8_t errorCode, unsigned int extraData);

	// Pipe has new message, read it and process it.
	// Returns false when pipe receives termination message or has an error
	bool ReadPipeMessagesIntoQueue(); // Returns false on error
	bool ReadAndProcessPipeMessages(); // Returns false when thread must be terminated
	void ProcessPipeMessage(PipeMsgBase* pMsg, bool& outTerminate);

	// Appens message to pipe queue
	void SendPipeMessage(PipeMsgBase* pMsg);

	// Helper functions
	// Sends all data. Handles calling send() many times if it didn't send instantly.
	bool SendAll(const char* pData, unsigned int nBytes);
	bool SendMessage(ControlPanelProtocol::MessageType msgType, unsigned int payloadSize, const char* pPayload);
};
