#include "SensorInterface.h"

#include "../Common/CommonIntTypes.h"
#include "../Common/MathUtils.h"

#include <thread>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h> 
#include <fcntl.h>
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions

// GPIO index of motor control pin
// WIP might be in config
const uint MOTOR_CONTROL_GPIO = 21;

SensorInterface::SensorInterface()
{
	m_PipeFd[0] = -1;
	m_PipeFd[1] = -1;
	
	m_RawScanSampleBuffer.reserve(Sensor::DEFAULT_SAMPLE_BUFFER_SIZE);
	m_OutScanSamples.reserve(Sensor::DEFAULT_SAMPLE_BUFFER_SIZE);
	m_OutIntScanSamples.reserve(Sensor::DEFAULT_SAMPLE_BUFFER_SIZE);
}

SensorInterface::~SensorInterface()
{
	ClosePipe();
	CloseSerialPort();
	SetMotorState_mt(false);
}

void SensorInterface::ClosePipe()
{
	if (m_PipeFd[0] != -1)
		close(m_PipeFd[0]);
	if (m_PipeFd[1] != -1)
		close(m_PipeFd[1]);
	m_PipeFd[0] = -1;
	m_PipeFd[1] = -1;
}

void SensorInterface::CloseSerialPort()
{
	if (m_SerialPortFd != -1)
		close(m_SerialPortFd);
	m_SerialPortFd = -1;
}

bool SensorInterface::SetMotorState_mt(bool enable)
{
	std::unique_lock lock(m_GpioMutex);

	if (enable)
	{
		return m_Gpio.SetPullUpDown(MOTOR_CONTROL_GPIO, Pigpio::PullUpDown::OFF) &&
			m_Gpio.SetMode(MOTOR_CONTROL_GPIO, Pigpio::Mode::OUTPUT) &&
			m_Gpio.Write(MOTOR_CONTROL_GPIO, true);
	}
	else
	{
		return m_Gpio.Write(MOTOR_CONTROL_GPIO, false) &&
			m_Gpio.SetPullUpDown(MOTOR_CONTROL_GPIO, Pigpio::PullUpDown::DOWN) &&
			m_Gpio.SetMode(MOTOR_CONTROL_GPIO, Pigpio::Mode::INPUT);
	}
}

bool SensorInterface::GetMotorState_mt()
{
	std::unique_lock lock(m_GpioMutex);

	return m_Gpio.Read(MOTOR_CONTROL_GPIO);
}

bool SensorInterface::Start()
{
	if (m_bRunning)
		return false;

	// Init GPIO
	if (!m_Gpio.Start())
	{
		fprintf(stderr, "SensorInterface: GPIO initialization error, unable to start motor.\n");
		return false;
	}

	if (!SetMotorState_mt(true))
	{
		fprintf(stderr, "SensorInterface: error setting motor GPIO value\n");
		return false;
	}

	// Open the serial port. Change device path as needed
	const char serialDevicePath[] = "/dev/serial0";
	printf("SensorInterface: Initializing sensor. Serial device: %s\n", serialDevicePath);
	m_SerialPortFd = open(serialDevicePath, O_RDWR);

	if (m_SerialPortFd < 0)
	{
		fprintf(stderr, "SensorInterface: open() error\n");
		return false;
	}

	// Create new termios struct, we call it 'tty' for convention
	struct termios tty;

	// Read in existing settings, and handle any error
	if (tcgetattr(m_SerialPortFd, &tty) != 0) {
		fprintf(stderr, "SensorInterface: tcgetattr error: %i, %s\n", errno, strerror(errno));
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

	tty.c_cc[VTIME] = 2;    // Wait for up to X deciseconds, returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(m_SerialPortFd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "SensorInterface: from tcsetattr error: %i, %s\n", errno, strerror(errno));
		return false;
	}

	if (!InitSensor())
	{
		fprintf(stderr, "SensorInterface: failed to initialize sensor\n");
		return false;
	}

	printf("SensorInterface: Sensor has been initialized\n");


	// Create pipes
	if (pipe2(m_PipeFd, O_NONBLOCK) == -1)
	{
		fprintf(stderr, "Error: SensorInterface pipe creation error\n");
		return false;
	}

	m_pThread = std::unique_ptr<std::thread>(
		new std::thread(&SensorInterface::ThreadFunction, this)
	);

	m_bRunning = true;
	return true;
}


bool SensorInterface::InitSensor()
{
	uchar readBuffer[16];

	// Send STOP
	{
		const uchar rqStop[2] = { 0xA5, 0x25 };
		write(m_SerialPortFd, rqStop, 2);
	}

	// Wait (until it stops)
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// Flush read buffer
	while (true)
	{
		ssize_t nBytesRead = read(m_SerialPortFd, readBuffer, sizeof(readBuffer));
		if (nBytesRead == 0)
			break;
	}

	// Send GET HEALTH
	{
		const uchar rqGetHealth[] = { 0xA5, 0x52 };
		write(m_SerialPortFd, rqGetHealth, 2);
	}

	// Read GET HEALTH response
	{
		memset(readBuffer, 0, sizeof(readBuffer));
		ssize_t nBytesReadTtl = 0;
		while (nBytesReadTtl != 10)
		{
			ssize_t nBytesRead = read(m_SerialPortFd, &readBuffer[nBytesReadTtl], 10);
			if (nBytesRead == 0)
				break; // If nothing arrived during timeout, it's a comms error
			nBytesReadTtl += nBytesRead;
		}
		if (nBytesReadTtl != 10)
		{
			fprintf(stderr, "SensorInterface: Did not receive GET_HEALTH response\n");
			return false;
		}
		// Verify descriptor
		const uchar descriptorGetHealth[7] = { 0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06 };
		if (memcmp(descriptorGetHealth, readBuffer, sizeof(descriptorGetHealth)) != 0)
		{
			fprintf(stderr, "SensorInterface: Error in GET_HEALTH response descriptor\n");
			return false;
		}
		// Verify data
		const uchar status = readBuffer[7];
		if (status != 0)
		{
			fprintf(stderr, "SensorInterface: Error in GET_HEALTH response: %i\n", static_cast<int>(status));
			return false;
		}
	}

	// Send SCAN
	{
		const uchar rqScan[] = { 0xA5, 0x20 };
		write(m_SerialPortFd, rqScan, 2);
	}

	// Read SCAN response descriptor
	{
		const uchar descriptorScan[7] = { 0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81 };
		memset(readBuffer, 0, sizeof(readBuffer));
		ssize_t nBytesReadTtl = 0;
		while (nBytesReadTtl < static_cast<ssize_t>(sizeof(descriptorScan)))
		{
			ssize_t nBytesRead = read(m_SerialPortFd, &readBuffer[nBytesReadTtl], sizeof(descriptorScan)); // !!! Switch it to 5
			if (nBytesRead == 0)
				break;
			nBytesReadTtl += nBytesRead;
		}
		if (nBytesReadTtl < static_cast < ssize_t>(sizeof(descriptorScan)))
		{
			fprintf(stderr, "SensorInterface: Did not receive SCAN response descriptor\n");
			return false;
		}
		if (memcmp(descriptorScan, readBuffer, sizeof(descriptorScan)) != 0)
		{
			fprintf(stderr, "SensorInterface: Error in SCAN response descriptor\n");
			return false;
		}

		// Process remaining bytes as scan data
		ssize_t nBytesRemaining = nBytesReadTtl - sizeof(descriptorScan);
		if (nBytesRemaining > 0)
		{
			ReadScanDataStream(&readBuffer[nBytesReadTtl], static_cast<uint>(nBytesRemaining));
		}
	}

	return true;
}

void SensorInterface::TerminateWait()
{
	if (!m_bRunning)
		return;

	uchar termCmd = PIPE_CMD_TERMINATE;
	write(m_PipeFd[1], &termCmd, 1);
	m_pThread->join();

	ClosePipe();
	CloseSerialPort();
	SetMotorState_mt(false);
}

void SensorInterface::ThreadFunction()
{
	// We don't want to accumulate too many samples before processing them.
	// One data packet is 5 bytes, that's why buffer size is 10.
	uchar readBuffer[10];
	fd_set readFds;
	const int pipeReadFd = m_PipeFd[0];

	bool loopRead = true;
	while (loopRead)
	{
		FD_ZERO(&readFds);
		FD_SET(pipeReadFd, &readFds);
		FD_SET(m_SerialPortFd, &readFds);
		int selectRes = select(std::max(pipeReadFd, m_SerialPortFd) + 1, &readFds, nullptr, nullptr, nullptr);
		if (selectRes == -1)
		{
			fprintf(stderr, "SensorInterface: select() error: %i\n", errno);
			return;
		}
		else
		{
			if (FD_ISSET(pipeReadFd, &readFds))
			{
				printf("SensorInterface: Received Terminate command\n");
				loopRead = false;
			}
			else if (FD_ISSET(m_SerialPortFd, &readFds))
			{
				ssize_t nBytesRead = read(m_SerialPortFd, readBuffer, sizeof(readBuffer));
				if (nBytesRead > 0)
					ReadScanDataStream(readBuffer, static_cast<uint>(nBytesRead));
			}
		}
	}

	// Send STOP
	{
		const uchar rqStop[2] = { 0xA5, 0x25 };
		write(m_SerialPortFd, rqStop, 2);
	}

	// Wait (until it stops)
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// Flush read buffer
	while (true)
	{
		ssize_t nBytesRead = read(m_SerialPortFd, readBuffer, sizeof(readBuffer));
		if (nBytesRead == 0)
			break;
	}

	close(m_SerialPortFd);
	m_SerialPortFd = -1;

	printf("SensorInterface: Thread terminated\n");
}

void SensorInterface::ReadScanDataStream(const uchar* pData, uint nBytes)
{
	for (uint i = 0; i < nBytes; i++)
	{
		uchar b = pData[i];
		if (m_PacketByteId == 0)
		{
			// BYte 0 in packet: Quality[7..2], !S, S
			uchar startBits = b & 0b11;
			if (startBits == 0b01 || startBits == 0b10)
			{
				memset(m_ScanPacketBuffer, 0, sizeof(m_ScanPacketBuffer));
				m_ScanPacketBuffer[0] = b;
				m_PacketByteId = 1;
			}
			else
			{
				fprintf(stderr, "SensorInterface: Error in start flags of data packet\n");
				m_PacketByteId = 0;
			}
		}
		else
		{
			m_ScanPacketBuffer[m_PacketByteId] = b;
			m_PacketByteId++;

			if (m_PacketByteId == 5)
			{
				// Process full packet
				if (m_ScanPacketBuffer[1] & 1)
				{
					// Check bit is good, process the packet
					ProcessScanDataPacket();
				}
				else
				{
					fprintf(stderr, "SensorInterface: Error in check bit of data packet\n");
				}

				m_PacketByteId = 0;
			}
		}
	}
}

void SensorInterface::ProcessScanDataPacket()
{
	// For sanity, treat them as unsigned chars
	const uchar* p = &m_ScanPacketBuffer[0];

	// Did it have start flag?
	const uchar startBits = p[0] & 0b11;
	const bool startOfNewScan = startBits == 0b01;

	// Ignore packet if we are still waiting for first scan start,
	// because this is trailing data of some previous scan
	if (!startOfNewScan && m_WaitingFirstScanStart)
		return;

	if (startOfNewScan)
	{
		if (!m_WaitingFirstScanStart)
		{
			ProcessScanResult();
		}
		m_WaitingFirstScanStart = false;
		m_RawScanSampleBuffer.clear();
	}

	if (m_RawScanSampleBuffer.size() < 1024)
	{
		// Decode packet and store it

		RawScanSample s;

		s.m_RawAngle = ((static_cast<uint>(p[1]) >> 1) & 0b01111111) |
			(static_cast<uint>(p[2]) << 7);
		s.m_RawDistance = static_cast<uint>(p[3]) | (static_cast<uint>(p[4]) << 8);
		s.m_Quality = (p[0] >> 2) & 0b00111111;

		m_RawScanSampleBuffer.push_back(s);
	}
	else
	{
		fprintf(stderr, "SensorInterface: Accumulated too many samples, something is wrong\n");
	}
}

void SensorInterface::ProcessScanResult()
{
	// m_pScanSampleBuffer contains scan data, convert it and invoke event for subscriber

	m_OutScanSamples.reserve(m_RawScanSampleBuffer.size());
	m_OutScanSamples.clear();
	m_OutIntScanSamples.reserve(m_RawScanSampleBuffer.size());
	m_OutIntScanSamples.clear();
	for (RawScanSample& rawSample : m_RawScanSampleBuffer)
	{
		if (rawSample.m_RawDistance == 0 || rawSample.m_Quality == 0)
			continue;

		Sensor::ScanSample outSample;
		Sensor::ConvertSampleIntToFloat(rawSample.m_RawAngle, rawSample.m_RawDistance, outSample.m_Angle_rad, outSample.m_Dist_m);
		m_OutScanSamples.push_back(outSample);

		Sensor::ScanSampleInt outSampleInt;
		outSampleInt.m_Angle = rawSample.m_RawAngle;
		outSampleInt.m_Dist = rawSample.m_RawDistance;
		m_OutIntScanSamples.push_back(outSampleInt);
	}

	// printf("SensorInterface: Received scan: total samples: %i, valid samples: %i\n", m_ScanSampleId, outSampleId);

	for (uint i = 0; i < m_OnReceivedScan.size(); i++)
	{
		m_OnReceivedScan[i](m_OutScanSamples, m_OutIntScanSamples);
	}
}
