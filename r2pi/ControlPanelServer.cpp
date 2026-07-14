#include "ControlPanelServer.h"
#include "../Common/ControlPanelProtocol.h"
#include "SensorInterface.h"

#include <thread>
#include <algorithm>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

bool ControlPanelServer::Start()
{
	if (m_bRunning)
		return true;

	// Create pipes
	// O_DIRECT - Pipe works in packet mode
	if (pipe2(m_PipeFd, O_NONBLOCK | O_DIRECT) == -1)
	{
		fprintf(stderr, "Error: ControlPanelServer pipe creation error\n");
		return false;
	}

	m_pThread = std::unique_ptr<std::thread>(
		new std::thread(&ControlPanelServer::ThreadFunction, this)
	);

	m_bRunning = true;
	return true;
}

void ControlPanelServer::TerminateWait()
{
	if (!m_bRunning)
		return;

	PipeMsgTerminate* pMsg = new PipeMsgTerminate();
	PostPipeMessage(pMsg);
	m_pThread->join();

	ClosePipe();
}

bool ControlPanelServer::GetConnected() const
{
	return m_bRunning && m_ClientSockFd > 0;
}

bool ControlPanelServer::GetStreamSensorData() const
{
	return m_bStreamSensorData;
}

ControlPanelServer::ControlPanelServer(MainMessageLoop& mainMsgLoop, SensorInterface& sensorInterface,
	RobotCore& robotCore)
	: m_pMainMsgLoop(&mainMsgLoop),	m_pSensorInterface(&sensorInterface), m_pRobotCore(&robotCore)
{
	m_PipeFd[0] = -1;
	m_PipeFd[1] = -1;
}

ControlPanelServer::~ControlPanelServer()
{
	ClosePipe();
}

void ControlPanelServer::ThreadFunction()
{
	// Common variables
	// Read buffer for processing TCP data
	char readBuffer[1024];
	ControlPanelProtocol::MessageDecoder msgDecoder;
	msgDecoder.BindOnReceivedMessage(
		[this](uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
		{ OnProtocolNewMessage(msgType, msgPayload, msgPayloadSize); }
	);
	msgDecoder.BindOnError(
		[this](uint8_t errorCode, unsigned int extraData)
		{ OnProtocolError(errorCode, extraData); }
	);

	// Create socket
	int sockFd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockFd < 0)
	{
		fprintf(stderr, "Error: ControlPanelServer socket() failed\n");
		return;
	}

	// Bind socket
	struct sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	serverAddr.sin_port = htons(ControlPanelProtocol::CONTROL_PANEL_PORT);
	if (bind(sockFd,
		reinterpret_cast<sockaddr*>(&serverAddr),
		sizeof(serverAddr)) < 0)
	{
		fprintf(stderr, "Error: ControlPanelServer bind() failed\n");
		return;
	}
	printf("ControlPanelServer: Opened socket on port %i\n", ControlPanelProtocol::CONTROL_PANEL_PORT);

	// Listen for connections
	listen(sockFd, 5);

	bool loopAccept = true;
	while (loopAccept)
	{
		printf("ControlPanelServer: Waiting for TCP connection\n");

		// wait for new connection or termination pipe
		fd_set readFds;
		const int pipeReadFd = m_PipeFd[0];

		FD_ZERO(&readFds);
		FD_SET(pipeReadFd, &readFds);
		FD_SET(sockFd, &readFds);
		int selectRes = select(std::max(pipeReadFd, sockFd) + 1, &readFds, nullptr, nullptr, nullptr);

		if (selectRes == -1)
		{
			fprintf(stderr, "ControlPanelServer: select() error: %i\n", errno);
			loopAccept = false;
		}
		else if (FD_ISSET(pipeReadFd, &readFds))
		{
			if (!ReadAndProcessPipeMessages())
			{
				printf("ControlPanelServer: Received Terminate command\n");
				loopAccept = false;
			}
		}
		else if (FD_ISSET(sockFd, &readFds))
		{
			struct sockaddr_in clientAddr;
			socklen_t sizeof_sockaddr_in = sizeof(clientAddr);

			m_ClientSockFd = accept(sockFd,
				reinterpret_cast<sockaddr*>(&clientAddr),
				&sizeof_sockaddr_in);

			if (m_ClientSockFd < 0)
			{
				fprintf(stderr, "ControlPanelServer: accept() failed: %i\n", errno);
			}
			else
			{
				char clientAddrStr[32];
				clientAddrStr[31] = 0; // for safety
				inet_ntop(clientAddr.sin_family, &clientAddr.sin_addr.s_addr, clientAddrStr, sizeof(clientAddrStr));
				printf("ControlPanelServer: Accepted connection from: %s:%i\n", clientAddrStr, ntohs(clientAddr.sin_port));

				// Reset msg decoder! New connection must be treated regardless of what was sent before.
				msgDecoder.Reset();

				bool loopRead = true;
				while (loopRead)
				{
					FD_ZERO(&readFds);
					FD_SET(pipeReadFd, &readFds);
					FD_SET(m_ClientSockFd, &readFds); // note here we use client FD!
					selectRes = select(std::max(pipeReadFd, m_ClientSockFd) + 1, &readFds, nullptr, nullptr, nullptr);
					
					if (selectRes == -1)
					{
						fprintf(stderr, "ControlPanelServer: select() error: %i\n", errno);
						loopRead = false;
					}
					else
					{
						if (FD_ISSET(pipeReadFd, &readFds))
						{
							if (!ReadAndProcessPipeMessages())
							{
								printf("ControlPanelServer: Received Terminate command\n");
								loopAccept = false;
								loopRead = false;
							}
						}
						else if (FD_ISSET(m_ClientSockFd, &readFds))
						{
							memset(readBuffer, 0, sizeof(readBuffer));
							ssize_t nBytesRead = read(m_ClientSockFd, readBuffer, sizeof(readBuffer));
							
							if (nBytesRead == 0)
							{
								printf("ControlPanelServer: Connection was terminated by client\n");
								loopRead = false;
							}
							else if (nBytesRead < 0)
							{
								fprintf(stderr, "ControlPanelServer: read() error: %i\n", errno);
								loopRead = false;
							}
							else
							{
								msgDecoder.ReadStream(readBuffer, static_cast<unsigned int>(nBytesRead));
							}
						}
					}

					
				} // while loopRead

				close(m_ClientSockFd);
				m_ClientSockFd = -1;
			}
		}
	} // while loopAccept

	close(sockFd);
	printf("ControlPanelServer: Thread terminated\n");
}

// Reads all pipe messages and puts them into queue
bool ControlPanelServer::ReadPipeMessagesIntoQueue()
{
	while (true)
	{
		PipeMsgBase* pMsg = nullptr;
		ssize_t nBytesRead = read(m_PipeFd[0], &pMsg, sizeof(PipeMsgBase*));
		int errorId = errno;

		// No more data in the pipe
		if (nBytesRead == 0 || (nBytesRead == -1 && errorId == EAGAIN))
			return true; // no error

		if (nBytesRead != sizeof(PipeMsgBase*))
		{
			fprintf(stderr, "ControlPanelServer: pipe read error: %i\n", errorId);
			return false;
		}

		m_PipeRxQueue.push_back(pMsg);
	}

	return true;
}

// Called on pipe new message, also handles termination of thread
bool ControlPanelServer::ReadAndProcessPipeMessages()
{
	while (true)
	{
		if (!ReadPipeMessagesIntoQueue())
			return false;

		// Select highest priority message
		if (m_PipeRxQueue.empty())
			return true;
		uint highestPrio = m_PipeRxQueue[0]->m_Priority;
		uint highestPrioMsgId = 0;
		for (uint i = 0; i < static_cast<uint>(m_PipeRxQueue.size()); i++)
		{
			if (m_PipeRxQueue[i]->m_Priority > highestPrio)
			{
				highestPrioMsgId = i;
				highestPrio = m_PipeRxQueue[i]->m_Priority;
			}
		}

		PipeMsgBase* pHighestPrioMsg = m_PipeRxQueue[highestPrioMsgId];

		// Process the message
		bool terminateThread = false;
		ProcessPipeMessage(pHighestPrioMsg, terminateThread);
		m_PipeRxQueue.erase(m_PipeRxQueue.begin() + highestPrioMsgId);

		// Remember to delete message!
		delete pHighestPrioMsg; // Virtual destructor!

		if (terminateThread)
			return false;

		// After processing each message, read more, in case more data has accumulated
	}
	return true;
}

void ControlPanelServer::ClosePipe()
{
	if (m_PipeFd[0] != -1)
		close(m_PipeFd[0]);
	if (m_PipeFd[1] != -1)
		close(m_PipeFd[1]);
	m_PipeFd[0] = -1;
	m_PipeFd[1] = -1;
}

void ControlPanelServer::PostPipeMessage(PipeMsgBase* pMsg)
{
	if (m_PipeFd[1] < 0)
		return;

	// Write raw pointer to pipe
	write(m_PipeFd[1], &pMsg, sizeof(PipeMsgBase*));
}

void ControlPanelServer::OnProtocolError(uint8_t errorCode, unsigned int extraData)
{
	fprintf(stderr, "ControlPanelServer: protocol error: %i, %i\n ", static_cast<int>(errorCode), static_cast<int>(extraData));
}

bool ControlPanelServer::SendAll(const char* pData, unsigned int nBytes)
{
	if (nBytes == 0)
		return false;

	const char* pDataLeft = pData;
	ssize_t nBytesLeft = static_cast<ssize_t>(nBytes);

	while (nBytesLeft > 0)
	{
		ssize_t nBytesSent = send(m_ClientSockFd, static_cast<const void*>(pDataLeft), nBytesLeft, 0);
		if (nBytesSent == -1)
			return false;

		nBytesLeft -= nBytesSent;
		pDataLeft += nBytesSent;
	}
	return true;
}

bool ControlPanelServer::SendMessage(ControlPanelProtocol::MessageType msgType, unsigned int payloadSize, const char* pPayload)
{
	ControlPanelProtocol::MessageHeader header;
	header.Init(msgType, payloadSize);

	// Send header
	if (!SendAll(reinterpret_cast<const char*>(&header), static_cast<size_t>(ControlPanelProtocol::MessageHeader::SIZE_PACKED)))
		return false;

	if (payloadSize != 0 && pPayload != nullptr)
	{
		if (!SendAll(pPayload, static_cast<size_t>(header.m_PayloadSize)))
			return false;
	}

	return true;
}


void ControlPanelServer::SendSensorData_mt(uint timestamp_ms, const std::vector<Sensor::ScanSampleInt>& scanInt,
	Chassis::TOdometerCounter odometer, Chassis::MoveState moveState, float steer)
{
	if (!GetConnected() || !GetStreamSensorData())
		return;

	// Called from Main thread!
	// Copy data and send it into pipe. The rest is handled in our own thread.

	PipeMsgSensorData* pMsg = new PipeMsgSensorData();
	pMsg->m_ScanSamples = scanInt;
	pMsg->m_Odometer = odometer;
	pMsg->m_MoveState = moveState;
	pMsg->m_Timestamp_ms = timestamp_ms;
	pMsg->m_Steer = steer;

	PostPipeMessage(pMsg);
}

void ControlPanelServer::SendMap_mt(const OccupancyMap& map)
{
	if (!GetConnected())
		return;

	PipeMsgMap* pMsg = new PipeMsgMap();
	pMsg->m_Map = map;

	PostPipeMessage(pMsg);
}


void ControlPanelServer::SendLocaction_mt(const Pose& meanPose, const std::vector<Pose>& poses)
{
	if (!GetConnected())
		return;

	PipeMsgLoc* pMsg = new PipeMsgLoc();
	pMsg->m_MeanPose = meanPose;
	pMsg->m_Poses = poses;

	PostPipeMessage(pMsg);
}

void ControlPanelServer::SendOptions_mt()
{
	if (!GetConnected())
		return;

	// Values of options will be gathered in own thread, not here
	PipeMsgSendOptions* pMsg = new PipeMsgSendOptions();
	PostPipeMessage(pMsg);
}

void ControlPanelServer::SendPath_mt(const Vector2& destPos, const std::vector<Vector2>& path)
{
	if (!GetConnected())
		return;

	PipeMsgPath* pMsg = new PipeMsgPath();
	pMsg->m_DestinationPos = destPos;
	pMsg->m_PathPoints = path;
	PostPipeMessage(pMsg);
}

void ControlPanelServer::SendDiagData_mt(const ControlPanelProtocol::DiagDataHeader& diagData)
{
	if (!GetConnected())
		return;

	PipeMsgDiagData * pMsg = new PipeMsgDiagData();
	pMsg->m_DiagData = diagData;
	PostPipeMessage(pMsg);
}