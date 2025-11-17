#pragma once

#include "../Common/Common.h"

#include <mutex>

// This class wraps serial communications with chassis motor controller.
// It does not run its own thread.
// Each SendCommand_XXX call is blocking and waits for response from the device.
class ChassisInterface
{
private:
	constexpr static uint MESSAGE_SIZE_MAX = 64;
	static const char MSG_ACK[];
	static const char STR_UNEXPECTED_RESPONSE[];

public:
	~ChassisInterface();
	bool Init();
	bool Terminate();

	std::mutex& GetMutex();

	bool SendCommand_Ping();
	bool SendCommand_MotorMove(float freq_Hz);
	bool SendCommand_MotorBrake();
	bool SendCommand_Steer(float steerNorm);
	
private:
	std::mutex m_Mutex;
	int m_SerialPortFd = -1;
	bool m_bInitSuccess = false;

	// Sends command and verifies that response matches
	bool SendCommand_VerifyResponse(const char* pCmd, const char* pExpectedResponse);

	// Reads response from the device, which is expected after every command sent to it.
	bool ReadResponse(char* msgBuffer, uint msgBufferSize, uint& outMessageSize);

	void CloseSerialPort();
};