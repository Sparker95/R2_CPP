#include "MainMessageLoop.h"

#ifdef TRACY_ENABLE
#include "Tracy/tracy/Tracy.hpp"
#else
#include "../Common/TracyPlug.h"
#endif

#include "SensorInterface.h"
#include "ChassisInterface.h"
#include "ControlPanelServer.h"

#include "../Common/RobotCore.h"
#include "../Common/ControlSensorFrame.h"
#include "../Common/Localizer.h"
#include "../Common/PathPlanner.h"
#include "../Common/PathFollower.h"

#include <stdio.h>
#include <cmath>

//------------------------------------------------------------------------------------------------------------------------
// Constructor, destructor

MainMessageLoop::MainMessageLoop(SensorInterface& sensorInterface, ChassisInterface& chassisInterface,
	const ChassisSettings::ControlSettings& chassisCtrlSettings,
	RobotCore& robotCore)
{
	m_pSensorInterface = &sensorInterface;
	m_pChassisInterface = &chassisInterface;
	m_ChassisCtrlSettings = chassisCtrlSettings;
	m_pRobotCore = &robotCore;

	m_pSensorInterface->BindOnReceivedScan(
		[this](const std::vector<Sensor::ScanSample>& scanSamples, const std::vector<Sensor::ScanSampleInt>& scanSamplesInt,
			SensorInterface::TTimestamp timeEnd, float scanPeriod_s)
		{ OnReceivedScan_mt(scanSamples, scanSamplesInt, timeEnd, scanPeriod_s); } );

	m_TimeStart = std::chrono::high_resolution_clock::now();
}

void MainMessageLoop::Init(ControlPanelServer* pCtrlPanelServer)
{
	m_pCtrlPanelServer = pCtrlPanelServer;
}

MainMessageLoop::~MainMessageLoop()
{
}

//------------------------------------------------------------------------------------------------------------------------
// Message loop

void MainMessageLoop::Execute()
{
	while (1)
	{
		std::unique_lock queueLock(m_QueueMutex);

		if (m_MessageQueue.empty())
		{
			m_QueueCondVariable.wait_for(queueLock, std::chrono::milliseconds(250));
			// now queueLock is locked
		}

		// Following cases are possible:
		// - It was not empty
		// - It was empty and got pushed
		// - It was empty and cv timed out
		// Either way check termination signal
		if (m_SignalValue.load() != 0)
		{
			printf("MainMessageLoop: Received signal from OS: %i\n", m_SignalValue.load());
			break;
		}

		if (m_MessageQueue.empty())
			continue;

		std::unique_ptr<MessageBase> pMsg = std::move(m_MessageQueue.front());
		m_MessageQueue.pop();
		queueLock.unlock(); // Unlock queue sooner, don't hold it while processing messaage

		// Process message
		{
			// The mutex is locked whenever we are processing message,
			// So that other threads could read safely our data
			std::unique_lock stateLock(m_StateMutex);
			ProcessMessage(*pMsg.get());
		}
	}

	printf("MainMessageLoop: returned from Execute()\n");
}

void MainMessageLoop::PostMessage(std::unique_ptr<MessageBase>&& pMsg)
{
	std::unique_lock queueLock(m_QueueMutex);
	m_MessageQueue.push(std::move(pMsg));
	m_QueueCondVariable.notify_one();
}

void MainMessageLoop::HandleOsSignal(int signalValue)
{
	m_SignalValue = signalValue;
}

//------------------------------------------------------------------------------------------------------------------------
// Message processing

void MainMessageLoop::ProcessMessage(MessageBase& msgBase)
{
	if (MessageNewSensorScanData* pMsg = dynamic_cast<MessageNewSensorScanData*>(&msgBase))
	{
		ControlSensorFrame& frame = m_CtrlSensorFrame; // Reuse same frame

		frame.m_ScanSamples = pMsg->m_Scan;

		// WIP Now we store only the timestamp of scan end
		frame.m_TimeScanEnd_ms = static_cast<uint>(std::chrono::duration_cast<std::chrono::milliseconds>(pMsg->m_TimeEnd - m_TimeStart).count());

		//printf("MainMessageLoop: Received new scan: %i raw samples\n", pMsg->m_RawSampleCount);

		// Read chassis data
		using TMoveState = ChassisInterface::MoveState;

		{
			ZoneScopedN("ChassisInterface_ReadValues");
			std::unique_lock chassisLock(m_pChassisInterface->GetMutex());
			m_pChassisInterface->SendCommand_GetOdometer(frame.m_Odometer, frame.m_MoveState);
			m_pChassisInterface->SendCommand_GetSteer(frame.m_Steer);
		}

		if (m_pCtrlPanelServer->GetConnected() && m_pCtrlPanelServer->GetStreamSensorData())
		{
			// Scan data is sent in integer format, to save bandwidth
			m_pCtrlPanelServer->SendSensorData_mt(frame.m_TimeScanEnd_ms, pMsg->m_ScanInt, frame.m_Odometer, frame.m_MoveState, frame.m_Steer);
		}

		// Pass data to RobotCore
		m_pRobotCore->Update(frame);

		// Send brake command, if crash detector was triggered
		// WIP it's not nice that the command is delayed by duration of Localizer update in RobotCore
		if (m_pRobotCore->GetBrakeOverride_mt())
		{
			{
				std::unique_lock chassisLock(m_pChassisInterface->GetMutex());
				m_pChassisInterface->SendCommand_MotorBrake();
			}
			printf("%u ms, ! Warning ! Incoming collision detected!\n", frame.m_TimeScanEnd_ms);
		}
		else if (m_pRobotCore->GetAutoControlsMode_mt())
		{
			const RobotCore::OutChassisControls& ctrl = m_pRobotCore->GetOutControls();
			{
				std::unique_lock chassisLock(m_pChassisInterface->GetMutex());

				float steerLimited = std::clamp(ctrl.m_Steer, -m_ChassisCtrlSettings.m_SteerOutputMax, m_ChassisCtrlSettings.m_SteerOutputMax);

				//m_pChassisInterface->SendCommand_SetSteerSpeed(32.0f); // WIP set instant steering speed
				m_pChassisInterface->SendCommand_Steer(steerLimited);

				if (ctrl.m_Brake)
					m_pChassisInterface->SendCommand_MotorBrake();
				else
					m_pChassisInterface->SendCommand_MotorMove(ctrl.m_MotorSpeed_Hz);
			}
		}

		// Send diagnostics data
		if (m_pCtrlPanelServer->GetConnected())
		{
			ControlPanelProtocol::DiagDataHeader diagData;

			diagData.m_MeanPose = m_pRobotCore->GetLocalizer()->GetMeanPose();

			const auto* pathFollower = m_pRobotCore->GetPathFollower();
			diagData.m_PathPointId = pathFollower->GetCurrentPointId();
			diagData.m_PathEndPointId = pathFollower->GetCurrentPointId();
			const auto& path = pathFollower->GetPath();

			if (diagData.m_PathPointId != (uint)-1)
				diagData.m_PathPointPos = path[diagData.m_PathPointId].m_Pos;

			if (diagData.m_PathEndPointId != (uint)-1)
				diagData.m_PathEndPointPos = path[diagData.m_PathEndPointId].m_Pos;

			diagData.m_PathLookaheadPos = pathFollower->GetLookaheadPos();

			m_pCtrlPanelServer->SendDiagData_mt(diagData);
		}

		// Print debug text
		if (m_SensorScanCounter % 30 == 0)
		{
			const char* moveStateText = "";
			switch (frame.m_MoveState)
			{
			case TMoveState::BACKWARD: moveStateText = "BWD"; break;
			case TMoveState::FORWARD: moveStateText = "FWD"; break;
			case TMoveState::STOP: moveStateText = "STOP"; break;
			}
			printf("%u ms, Scan Count: %u, Scan Frequency: %.2f Hz, Scan Size: %u, Odometer: %u, MoveState: %s, Steer: %.2f\n",
				frame.m_TimeScanEnd_ms, m_SensorScanCounter, 1/pMsg->m_ScanPeriod_s, pMsg->m_Scan.size(), static_cast<int>(frame.m_Odometer), moveStateText, frame.m_Steer);
		}

		m_SensorScanCounter++;

		FrameMark;

		return;
	}
	else if (MessageControlPanel* pMsg = dynamic_cast<MessageControlPanel*>(&msgBase))
	{
		// One of 'trivial' messages from control panel
		using TCtrlPanelMsgType = ControlPanelProtocol::MessageType;
		switch (pMsg->m_Type)
		{
			case TCtrlPanelMsgType::MSG_REQUEST_MAP:
			{
				// Send back a map

				/*
				// Fills random map
				uint xSize = 256; uint ySize = 256;
				Map m(xSize, ySize, 128, 128, 0.0625);
				for (uint y = 0; y < ySize; y++)
				{
					for (uint x = 0; x < xSize; x++)
					{
						m.SetDataAtPos(Map::TCellPos(x, y), static_cast<Map::TCellData>((8 * (x + y + m_SensorScanCounter)) & 0xFF));
					}
				}
				*/

				// Send common map or most likely map, depends on mode

				auto locMode = m_pRobotCore->GetLocalizerMode_mt();

				if (locMode == Localizer::Mode::LOC_AND_MAP)
				{
					const Map& mostLikelyMap = m_pRobotCore->GetLocalizer()->GetMostLikelyParticle().m_Map;
					m_pCtrlPanelServer->SendMap_mt(mostLikelyMap);
				}
				else if (locMode == Localizer::Mode::COMMON_LOC)
				{
					const Map& commonMap = m_pRobotCore->GetLocalizer()->GetLocalizationCommonMap();
					m_pCtrlPanelServer->SendMap_mt(commonMap);
				}

				break;
			}
			case TCtrlPanelMsgType::MSG_REQUEST_LOC:
			{
				// Send back location data
				std::vector<Pose> poses;

				const std::vector<Localizer::Particle>& particles = m_pRobotCore->GetLocalizer()->GetParticles();

				poses.resize(particles.size());

				for (uint i = 0; i < static_cast<uint>(particles.size()); i++)
					poses[i] = particles[i].m_Pose;

				/*
				// Fills random poses
				poses.reserve(256);
				for (uint i = 0; i < 256; i++)
				{
					uint rnd0 = (m_SensorScanCounter * 127 + i * 789) % 255;
					uint rnd1 = (m_SensorScanCounter * 611 + i * 1039) % 255;

					float x = static_cast<float>(rnd0) / 128.0f - 1.0f;
					float y = static_cast<float>(rnd1) / 128.0f - 1.0f;
					float dir = 0.0f;

					poses.push_back(Pose(x, y, dir));
				}
				*/

				const Pose& meanPose = m_pRobotCore->GetLocalizer()->GetMeanPose();

				m_pCtrlPanelServer->SendLocaction_mt(meanPose, poses);

				break;
			}
			case TCtrlPanelMsgType::MSG_ENABLE_LOCALIZER:
			{
				m_pRobotCore->SetUpdateLocalizer(pMsg->m_Payload[0] != 0);
				break;
			}
			case TCtrlPanelMsgType::MSG_RESET_LOCALIZER:
			{
				m_pRobotCore->ResetLocalizer();
				break;
			}
			case TCtrlPanelMsgType::MSG_SWITCH_LOCALIZER_MODE:
			{
				Localizer::Mode newMode = static_cast<Localizer::Mode>(pMsg->m_Payload[0]);
				m_pRobotCore->SwitchLocalizerMode(newMode);
				break;
			}
			case TCtrlPanelMsgType::MSG_ENABLE_AUTO_CONTROLS_MODE:
			{
				m_pRobotCore->SetAutoControlsMode(pMsg->m_Payload[0] != 0);
				break;
			}
			case TCtrlPanelMsgType::MSG_REQUEST_PATH:
			{
				PathPlanner* pPathPlanner = m_pRobotCore->GetPathPlanner();
				std::vector<Vector2> pathPoints;
				for (const auto& p : pPathPlanner->GetPath())
				{
					if (p.m_Type != PathFollower::PathPoint::Type::EXTENSION)
						pathPoints.push_back(p.m_Pos);
				}

				m_pCtrlPanelServer->SendPath_mt(pathPoints);
				break;
			}
			case TCtrlPanelMsgType::MSG_PLAN_PATH:
			{
				using THeader = ControlPanelProtocol::PlanPathHeader;
				THeader* header = reinterpret_cast<THeader*>(pMsg->m_Payload);

				PathPlanner* pPathPlanner = m_pRobotCore->GetPathPlanner();
				Localizer* pLocalizer = m_pRobotCore->GetLocalizer();

				const Localizer::Particle& locParticle = pLocalizer->GetMostLikelyParticle();

				pPathPlanner->PlanPath(locParticle.m_Map, locParticle.m_Pose, { header->m_X, header->m_Y }, header->m_Radius);

				if (pPathPlanner->PathValid())
				{
					m_pRobotCore->GetPathFollower()->FollowPath(pPathPlanner->GetPath());
				}

				break;
			}
			default:
			{
				fprintf(stderr, "MainMessageLoop: Received unknown Control Panel Protocol message type: %u",
					static_cast<uint>(pMsg->m_Type));
				break;
			}
		}

		return;
	}

	fprintf(stderr, "MainMessageLoop: Received unknown message type!\n");
}

//------------------------------------------------------------------------------------------------------------------------
// Connection with other objects

void MainMessageLoop::OnReceivedScan_mt(const std::vector<Sensor::ScanSample>& scanSamples,
	const std::vector<Sensor::ScanSampleInt>& scanSamplesInt,
	SensorInterface::TTimestamp timeEnd, float scanPeriod_s)
{
	MessageNewSensorScanData* pMsg = new MessageNewSensorScanData();

	pMsg->m_Scan = scanSamples;
	pMsg->m_ScanInt = scanSamplesInt;
	pMsg->m_TimeEnd = timeEnd;
	pMsg->m_ScanPeriod_s = scanPeriod_s;

	PostMessage(std::unique_ptr<MessageBase>(pMsg));
}
