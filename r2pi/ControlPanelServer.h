#pragma once

#include <memory>

#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"

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

private:
	constexpr static const char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread

	// Pipe message processing
	struct PipeMsgBase
	{
		virtual ~PipeMsgBase() {};
	};
	struct PipeMsgTerminate : PipeMsgBase
	{
	};
	struct PipeMsgSensorScanData : PipeMsgBase
	{
		std::vector<Sensor::ScanSample> m_Samples;
	};


	// Member variables:

	MainMessageLoop* m_pMainMsgLoop = nullptr;
	SensorInterface* m_pSensorInterface = nullptr;

	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	int m_ClientSockFd = -1;
	bool m_bRunning = false;
	bool m_bStreamSensorData = false;

	void ThreadFunction();
	void ClosePipe();

	void OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize);
	void OnProtocolError(uint8_t errorCode, unsigned int extraData);

	// Pipe has new message, read it and process it.
	// Returns false when pipe receives termination message or has an error
	bool ProcessPipeMessage();
	void OnPipeMessage(PipeMsgBase* pMsg, bool& outTerminate);

	// Appens message to pipe queue
	void SendPipeMessage(PipeMsgBase* pMsg);

	// Helper functions
	// Sends all data. Handles calling send() many times if it didn't send instantly.
	bool SendAll(const char* pData, unsigned int nBytes);
	bool SendMessage(ControlPanelProtocol::MessageType msgType, unsigned int payloadSize, const char* pPayload);

	// Called from SensorInterface thread!
	void OnSensorReceivedScan_mt(const Sensor::ScanSample* pSamples, uint nValidSamples, uint nRawSamples);

	// Other data
	std::vector<Sensor::ScanSample> m_IcpTestScan; // Used for ICP_TEST message
};
