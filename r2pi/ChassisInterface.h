#pragma once

#include "../Common/Pose.h"
#include "../Common/Chassis.h"

#include <mutex>
#include <cstdint>
#include <string>

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

	struct ConnectionSettings
	{
		std::string m_SerialDevPath;
	};

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

	struct SteeringSettings
	{
		float m_SteeringSpeed = 2.0f;
	};

	// Struct with all settings which can be sent to Motor Controller board
	struct AllSettings
	{
		ConnectionSettings m_ConnSettings;
		MotorCtrlPidSettings m_PidSettings;
		MotorCtrlAccelSettings m_AccelSettings;
		SteeringSettings m_SteeringSettings;
	};

	~ChassisInterface();
	bool Init(const AllSettings& settings);
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
	bool SendCommand_SetSteerSpeed(float steerSpeed);

private:
	std::mutex m_Mutex;
	int m_SerialPortFd = -1;
	bool m_bInitSuccess = false;

	// Sends command and verifies that response matches
	bool SendCommand_VerifyResponse(const char* pCmd, const char* pExpectedResponse);

	// Reads response from the device, which is expected after every command sent to it.
	bool ReadResponse(char* msgBuffer, uint msgBufferSize, uint& outMessageSize);

	// Writes command to device, waits until data has been transmitted
	void SendCommandSync(const char* pCmd);

	void CloseSerialPort();
};