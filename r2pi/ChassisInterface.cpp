#include "ChassisInterface.h"
#include "../Common/Common.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
//#include <stdlib.h>
//#include <sys/types.h> 
#include <fcntl.h>
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions

// Constants
const char ChassisInterface::MSG_ACK[] = "ACK;";
const char ChassisInterface::STR_UNEXPECTED_RESPONSE[] = "ChassisInterface: Unexpected response error: %s\n";


ChassisInterface::~ChassisInterface()
{
	CloseSerialPort();
}

std::mutex& ChassisInterface::GetMutex()
{
	return m_Mutex;
}

bool ChassisInterface::Init(const MotorCtrlPidSettings& pidSettings, const MotorCtrlAccelSettings& accelSettings)
{
    const char serialDevicePath[] = "/dev/ttyAMA3";
    printf("ChassisInterface: Initializing motor controller. Serial device: %s\n", serialDevicePath);

    m_SerialPortFd = open(serialDevicePath, O_RDWR);

    if (m_SerialPortFd < 0)
    {
        fprintf(stderr, "ChassisInterface: open() error\n");
        return false;
    }

	// Create new termios struct, we call it 'tty' for convention
	struct termios tty;

	// Read in existing settings, and handle any error
	if (tcgetattr(m_SerialPortFd, &tty) != 0) {
		fprintf(stderr, "ChassisInterface: tcgetattr error: %i, %s\n", errno, strerror(errno));
		return false;
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 1;    // Wait for up to X deciseconds, returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate
	cfsetispeed(&tty, B57600);
	cfsetospeed(&tty, B57600);

	// Save tty settings, also checking for error
	if (tcsetattr(m_SerialPortFd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "ChassisInterface: from tcsetattr error: %i, %s\n", errno, strerror(errno));
		return false;
	}

	// Reset communication state
	// This causes it to process leftover data in its receive buffer
	write(m_SerialPortFd, ";;", 2);

	// Flush read buffer
	while (true)
	{
		char tempBuffer[16];
		ssize_t nBytesRead = read(m_SerialPortFd, tempBuffer, sizeof(tempBuffer));
		if (nBytesRead == 0)
			break;
	}

	if (!SendCommand_Ping())
	{
		fprintf(stderr, "ChassisInterface: initialization error, motor controller does not respond\n");
		return false;
	}

	if (!SendCommand_SetPid(pidSettings))
	{
		fprintf(stderr, "ChassisInterface: initialization error, failed to send PID settings\n");
		return false;
	}

	if (!SendCommand_SetAccel(accelSettings))
	{
		fprintf(stderr, "ChassisInterface: initialization error, failed to send acceleration settings\n");
		return false;
	}

	printf("ChassisInterface: Motor Controller has been initialized\n");

	m_bInitSuccess = true;
	return true;
}

bool ChassisInterface::Terminate()
{
	CloseSerialPort();
	return true;
}

void ChassisInterface::CloseSerialPort()
{
	if (m_SerialPortFd != -1)
		close(m_SerialPortFd);
	m_SerialPortFd = -1;
}

bool ChassisInterface::ReadResponse(char* msgBuffer, uint msgBufferSize, uint& outMessageSize)
{
	char readBuffer[32];

	uint bufferWriteId = 0;

	while (true)
	{
		ssize_t nBytesRead = read(m_SerialPortFd, readBuffer, sizeof(readBuffer));
		if (nBytesRead == 0)
		{
			fprintf(stderr, "ChassisInterface: read() timeout error\n");
			return false; // Timeout
		}

		// Iterate the received bytes:
		// - Ignore all newlines, they are irrelevant
		// - ';' indicates end of message
		for (uint i = 0; i < static_cast<uint>(nBytesRead); i++)
		{
			char c = readBuffer[i];
			if (c == '\n')
				continue;
			if (bufferWriteId >= (msgBufferSize - 1)) // One character is left for 0 termination
			{
				fprintf(stderr, "ChassisInterface: message buffer overflow error\n");
				return false; // Overflow
			}
			msgBuffer[bufferWriteId] = c;
			bufferWriteId++;

			if (c == ';')
			{
				// This was the end of message
				msgBuffer[bufferWriteId] = 0;
				outMessageSize = bufferWriteId;
				return true;
			}
		}
	}
}


//--------------------------------------------------------------------
// Functions for sending specific commands

bool ChassisInterface::SendCommand_Ping()
{
	return SendCommand_VerifyResponse("PING;", "MT_CTRL;");
}

bool ChassisInterface::SendCommand_MotorMove(float freq_Hz)
{
	char cmd[MESSAGE_SIZE_MAX];
	sprintf(cmd, "MT_MV %.1f;", freq_Hz);
	return SendCommand_VerifyResponse(cmd, MSG_ACK);
}

bool ChassisInterface::SendCommand_MotorBrake()
{
	return SendCommand_VerifyResponse("MT_BR;", MSG_ACK);
}

bool ChassisInterface::SendCommand_Steer(float steerNorm)
{
	char cmd[MESSAGE_SIZE_MAX];
	sprintf(cmd, "ST %.3f;", steerNorm);
	return SendCommand_VerifyResponse(cmd, MSG_ACK);
}

bool ChassisInterface::SendCommand_SetPid(const MotorCtrlPidSettings& s)
{
	char cmd[MESSAGE_SIZE_MAX];
	sprintf(cmd, "SET_PID %.2f %.2f %.2f %.4f %.3f;",
		s.m_fKP, s.m_fKI, s.m_fKD, s.m_fErrorScale, s.m_fErrorIntegralMax);
	return SendCommand_VerifyResponse(cmd, MSG_ACK);
}

bool ChassisInterface::SendCommand_SetAccel(const MotorCtrlAccelSettings& s)
{
	char cmd[MESSAGE_SIZE_MAX];
	sprintf(cmd, "SET_ACCEL %.2f %.2f;",
		s.m_fAccelMove, s.m_fAccelBrake);
	return SendCommand_VerifyResponse(cmd, MSG_ACK);
}

bool ChassisInterface::SendCommand_VerifyResponse(const char* pCmd, const char* pExpectedResponse)
{
	write(m_SerialPortFd, static_cast<const void*>(pCmd), strlen(pCmd));

	char msgBuffer[MESSAGE_SIZE_MAX];
	uint msgSize = 0;
	if (!ReadResponse(msgBuffer, sizeof(msgBuffer), msgSize))
		return false;

	if (strcmp(msgBuffer, pExpectedResponse) == 0)
		return true;

	fprintf(stderr, STR_UNEXPECTED_RESPONSE, msgBuffer);
	return false;
}