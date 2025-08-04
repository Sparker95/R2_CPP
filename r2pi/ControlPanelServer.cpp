#include "ControlPanelServer.h"
#include "../Common/ControlPanelProtocol.h"

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

bool ControlPanelServer::Start()
{
	if (m_bRunning)
		return true;

	// Create pipes
	if (pipe2(m_PipeFd, O_NONBLOCK) == -1)
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

	char termCmd = PIPE_CMD_TERMINATE;
	write(m_PipeFd[1], &termCmd, 1);
	m_pThread->join();

	ClosePipe();
}

ControlPanelServer::ControlPanelServer()
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
		fprintf(stderr, "Error: ControlPanelServer socket() failed\n");

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
			printf("ControlPanelServer: Received Terminate command\n");
			loopAccept = false;
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
							printf("ControlPanelServer: Received Terminate command\n");
							loopAccept = false;
							loopRead = false;
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
								/*
								printf("ControlPanelServer: received data:");
								printf(readBuffer);
								printf(" - end\n");
								// process bytes
								*/
							}
						}
					}

					
				} // while loopRead

				close(m_ClientSockFd);
			}
		}
	} // while loopAccept

	close(sockFd);
	printf("ControlPanelServer: Thread terminated\n");
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

void ControlPanelServer::OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
{
	using MessageType = ControlPanelProtocol::MessageType;
	switch (static_cast<ControlPanelProtocol::MessageType>(msgType))
	{
	case (MessageType::MSG_PRINT):
	{
		// For safety, null terminate string
		if (msgPayloadSize < ControlPanelProtocol::MAX_PAYLOAD_SIZE - 1)
			msgPayload[msgPayloadSize] = 0;
		else
			msgPayload[ControlPanelProtocol::MAX_PAYLOAD_SIZE - 1] = 0;

		printf("Message: ");
		printf(msgPayload);
		printf("\n");

		break;
	}
	case (MessageType::MSG_TEST0):
	{
		// Send back some message
		ControlPanelProtocol::MessageHeader header;
		char text[] = "Hello from server!";
		header.Init(MessageType::MSG_PRINT, sizeof(text));
		send(m_ClientSockFd, static_cast<const void*>(&header), static_cast<size_t>(ControlPanelProtocol::MessageHeader::SIZE_PACKED), 0);
		send(m_ClientSockFd, static_cast<const void*>(text), static_cast<size_t>(header.m_PayloadSize), 0);
		break;
	}
	default:
		break;
	}
}

void ControlPanelServer::OnProtocolError(uint8_t errorCode, unsigned int extraData)
{
	fprintf(stderr, "ControlPanelServer: protocol error: %i, %i\n ", static_cast<int>(errorCode), static_cast<int>(extraData));
}
