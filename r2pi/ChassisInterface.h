#pragma once

#include "../Common/Common.h"
#include "../Common/Chassis.h"

#include <mutex>
#include <cstdint>

// This class wraps serial communications with chassis motor controller.
// It does not run its own thread.
// Each SendCommand_XXX call is blocking and waits for response from the device.
class ChassisInterface
{
private:
	constexpr static uint MESSAGE_SIZE_MAX = 64;

public:
	// Public types
	using TOdometerCounter = Chassis::TOdometerCounter;
	using MoveState = Chassis::MoveState;

	struct MotorCtrlPidSettings
	{
		float m_KP = 0.5f;
		float m_KI = 0.5f;
		float m_KD = 0.0f;
		float m_ErrorScale = 0.001f;
		float m_ErrorIntegralMax = 1.0f;
	};

	struct MotorCtrlAccelSettings
	{
		float m_AccelMove = 100.0f;
		float m_AccelBrake = 100.0f;
	};

	~ChassisInterface();
	bool Init(const MotorCtrlPidSettings& pidSettings, const MotorCtrlAccelSettings& accelSettings);
	bool Terminate();

	std::mutex& GetMutex();

	// !!! Remember to aquire lock when sending commands !!!
	// std::unique_lock chassisLock(m_pChassisInterface->GetMutex());
	bool SendCommand_Ping();
	bool SendCommand_MotorMove(float freq_Hz);
	bool SendCommand_MotorBrake();
	bool SendCommand_GetOdometer(TOdometerCounter& outOdometerCounter, MoveState& outMoveState);
	bool SendCommand_Steer(float steerNorm);
	bool SendCommand_GetSteer(float& outSteerNorm);
	bool SendCommand_SetPid(const MotorCtrlPidSettings& s);
	bool SendCommand_SetAccel(const MotorCtrlAccelSettings& s);
	
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