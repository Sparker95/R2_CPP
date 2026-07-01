#pragma once

#include <memory>
#include <vector>

#include "../Common/Vector2.h"
#include "../Common/Pose.h"
#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Chassis.h"
#include "../Common/Map.h"

namespace std
{
	class thread;
}

class MainMessageLoop;
class SensorInterface;
class RobotCore;

class ControlPanelServer
{
public:
	bool Start();
	void TerminateWait();
	ControlPanelServer(MainMessageLoop& mainMsgLoop,
		SensorInterface& sensorInterface,
		RobotCore& robotCore);
	~ControlPanelServer();

	bool GetConnected() const;
	bool GetStreamSensorData() const;

	// Called from Main thread!
	void SendSensorData_mt(uint timestamp_ms, const std::vector<Sensor::ScanSampleInt>& scan, Chassis::TOdometerCounter odometer,
		Chassis::MoveState moveState, float steer);

	void SendMap_mt(const OccupancyMap& map);
	void SendLocaction_mt(const Pose& meanPose, const std::vector<Pose>& poses);
	void SendOptions_mt(); // Sends all options back to client
	void SendPath_mt(const std::vector<Vector2> path);
	void SendDiagData_mt(const ControlPanelProtocol::DiagDataHeader& diagData);

private:
	constexpr static const char PIPE_CMD_TERMINATE = 0x78; // Sent into pipe to termiante the thread

	// Pipe message processing
	struct PipeMsgBase
	{
		uint m_Priority = 0;
		virtual ~PipeMsgBase() {};
	};
	struct PipeMsgTerminate : PipeMsgBase
	{
		PipeMsgTerminate() { m_Priority = 100; }
	};
	struct PipeMsgSensorData : PipeMsgBase
	{
		std::vector<Sensor::ScanSampleInt> m_ScanSamples;
		float m_Steer = 0;
		Chassis::TOdometerCounter m_Odometer = 0;
		Chassis::MoveState m_MoveState = {};
		uint m_Timestamp_ms = 0;

		PipeMsgSensorData() { m_Priority = 10; }
	};
	struct PipeMsgMap : PipeMsgBase
	{
		OccupancyMap m_Map;
	};
	struct PipeMsgLoc : PipeMsgBase
	{
		Pose m_MeanPose;
		std::vector<Pose> m_Poses;
	};
	struct PipeMsgPath : PipeMsgBase
	{
		std::vector<Vector2> m_PathPoints;
	};
	struct PipeMsgDiagData : PipeMsgBase
	{
		ControlPanelProtocol::DiagDataHeader m_DiagData;
	};
	struct PipeMsgSendOptions : PipeMsgBase {}; // Notifies server to gather options values and send them to client


	// Member variables:

	MainMessageLoop* m_pMainMsgLoop = nullptr;
	SensorInterface* m_pSensorInterface = nullptr;
	RobotCore* m_pRobotCore = nullptr;

	std::unique_ptr<std::thread> m_pThread;
	int m_PipeFd[2]; // 0 - read, 1 - write
	std::vector<PipeMsgBase*> m_PipeRxQueue; // Messages are taken from pipe into this array
	int m_ClientSockFd = -1;
	bool m_bRunning = false;
	bool m_bStreamSensorData = false;

	void ThreadFunction();
	void ClosePipe();

	void OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize);
	void OnProtocolError(uint8_t errorCode, unsigned int extraData);

	// Pipe has new message, read it and process it.
	// Returns false when pipe receives termination message or has an error
	bool ReadPipeMessagesIntoQueue(); // Returns false on error
	bool ReadAndProcessPipeMessages(); // Returns false when thread must be terminated
	void ProcessPipeMessage(PipeMsgBase* pMsg, bool& outTerminate);

	// Appens message to pipe queue
	void PostPipeMessage(PipeMsgBase* pMsg);

	// Helper functions
	// Sends all data. Handles calling send() many times if it didn't send instantly.
	bool SendAll(const char* pData, unsigned int nBytes);
	bool SendMessage(ControlPanelProtocol::MessageType msgType, unsigned int payloadSize, const char* pPayload);

	// Other

	// Fills data of options header, meant to be called in Server's thread.
	void FillOptionsValues(ControlPanelProtocol::OptionsHeader& optsHeader);
	void SendOptions(); // Sends options instantly !!! Not MT safe.
};
