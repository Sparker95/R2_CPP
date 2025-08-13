#pragma once

#include <memory>

#include "../Common/ControlPanelProtocol.h"

namespace std
{
	class thread;
}

class MainMessageLoop;

class ControlPanelServer
{
public:
	bool Start();
	void TerminateWait();
	ControlPanelServer(MainMessageLoop* pMainMsgLoop);
	~ControlPanelServer();

private:
	constexpr static const char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread

	MainMessageLoop* m_pMainMsgLoop = nullptr;
	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	int m_ClientSockFd = -1;
	bool m_bRunning = false;

	void ThreadFunction();
	void ClosePipe();

	void OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize);
	void OnProtocolError(uint8_t errorCode, unsigned int extraData);

	// Helper functions
	// Sends all data. Handles calling send() many times if it didn't send instantly.
	bool SendAll(const char* pData, unsigned int nBytes);
	bool SendMessage(ControlPanelProtocol::MessageType msgType, unsigned int payloadSize, const char* pPayload);
};
