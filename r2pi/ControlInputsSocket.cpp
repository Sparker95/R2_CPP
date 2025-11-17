#include "ControlInputsSocket.h"

#include "ChassisInterface.h"
#include "../Common/ControlInputsProtocol.h"

#include <thread>
#include <algorithm>
#include <mutex>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>


ControlInputsSocket::ControlInputsSocket(ChassisInterface* pChassisInterface)
	: m_pChassisInterface(pChassisInterface)
{
	m_PipeFd[0] = -1;
	m_PipeFd[1] = -1;
}

ControlInputsSocket::~ControlInputsSocket()
{
	ClosePipe();
}


bool ControlInputsSocket::Start()
{
	if (m_bRunning)
		return true;

	// Create pipes
	if (pipe2(m_PipeFd, O_NONBLOCK) == -1)
	{
		fprintf(stderr, "Error: ControlInputsSocket pipe creation error\n");
		return false;
	}

	m_pThread = std::unique_ptr<std::thread>(
		new std::thread(&ControlInputsSocket::ThreadFunction, this)
	);

	m_bRunning = true;
	return true;
}

void ControlInputsSocket::TerminateWait()
{
	if (!m_bRunning)
		return;

	char termCmd = PIPE_CMD_TERMINATE;
	write(m_PipeFd[1], &termCmd, 1);
	m_pThread->join();

	ClosePipe();
}

void ControlInputsSocket::ThreadFunction()
{
	// Read buffer for receiving UDP packets
	char readBuffer[1024];

	int sockFd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockFd < 0)
	{
		fprintf(stderr, "Error: ControlInputsSocket socket() failed\n");
		return;
	}

	// Bind socket
	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(ControlInputsProtocol::CONTROL_INPUTS_PORT);
	if (bind(sockFd,
		reinterpret_cast<sockaddr*>(&addr),
		sizeof(addr)) < 0)
	{
		fprintf(stderr, "Error: ControlInputsSocket bind() failed\n");
		return;
	}

	printf("ControlInputsSocket: Opened socket on port %i\n", ControlInputsProtocol::CONTROL_INPUTS_PORT);

	fd_set readFds;
	const int pipeReadFd = m_PipeFd[0];

	bool loopRead = true;
	while (loopRead)
	{
		FD_ZERO(&readFds);
		FD_SET(pipeReadFd, &readFds);
		FD_SET(sockFd, &readFds);
		int selectRes = select(std::max(sockFd, pipeReadFd) + 1, &readFds, nullptr, nullptr, nullptr);

		if (selectRes == -1)
		{
			fprintf(stderr, "Error: ControlInputsSocket select() error: %i\n", errno);
			loopRead = false;
		}
		else
		{
			if (FD_ISSET(pipeReadFd, &readFds))
			{
				printf("ControlInputsSocket: Received Terminate command\n");
				loopRead = false;
			}
			else if (FD_ISSET(sockFd, &readFds))
			{
				struct sockaddr_in senderAddr;
				socklen_t sizeof_senderAddr = sizeof(senderAddr);

				ssize_t nBytesRead = recvfrom(sockFd, readBuffer, sizeof(readBuffer), 0, (struct sockaddr*)&senderAddr, &sizeof_senderAddr);
				if (nBytesRead < 0)
					fprintf(stderr, "Error: ControlInputsSocket recvfrom() error: %i\n", errno);

				if (!ProcessMessage(readBuffer, static_cast<uint>(nBytesRead)))
					fprintf(stderr, "Error: ControlInputsSocket received invalid message");
			}
		}
	} // while loopRead
	
	close(sockFd);
	printf("ControlInputsSocket: Thread terminated\n");
}

bool ControlInputsSocket::ProcessMessage(const char* pMsg, uint msgSize)
{
	if (msgSize != sizeof(ControlInputsProtocol::Message))
		return false;

	const ControlInputsProtocol::Message* pControlInputsMsg =
		reinterpret_cast<const ControlInputsProtocol::Message*>(pMsg);

	if (!pControlInputsMsg->IsValid())
		return false;

	{
		std::unique_lock chassisLock(m_pChassisInterface->GetMutex());

		// WIP control limits
		const float STEER_MAX = 0.7f;
		const float MOVE_FREQ_MAX_HZ = 500.0f;

		if (pControlInputsMsg->m_bBrake)
			m_pChassisInterface->SendCommand_MotorBrake();
		else
		{
			float moveCmdValue = std::clamp(pControlInputsMsg->m_fMotorSpeed * MOVE_FREQ_MAX_HZ,
				-MOVE_FREQ_MAX_HZ, MOVE_FREQ_MAX_HZ);
			m_pChassisInterface->SendCommand_MotorMove(moveCmdValue);
		}

		float steerCmdValue = std::clamp(pControlInputsMsg->m_fSteer, -STEER_MAX, STEER_MAX);
		m_pChassisInterface->SendCommand_Steer(steerCmdValue);
	}

	return true;
}

void ControlInputsSocket::ClosePipe()
{
	if (m_PipeFd[0] != -1)
		close(m_PipeFd[0]);
	if (m_PipeFd[1] != -1)
		close(m_PipeFd[1]);
	m_PipeFd[0] = -1;
	m_PipeFd[1] = -1;
}
