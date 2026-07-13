#include "RobotCore.h"

#include "TracyPlug.h"

#include "Chassis.h"
#include "Localizer.h"
#include "CrashDetector.h"
#include "PathFollower.h"
#include "PathCoordinator.h"
#include "Explorer.h"

#include <cstdio>

RobotCore::RobotCore(const Chassis::ChassisModel& chassisModel, Localizer& localizer, CrashDetector& crashDetector,
	PathPlanner& pathPlanner, PathFollower& pathFollower, PathCoordinator& pathCoordinator, Explorer& explorer)
	: m_pChassisModel(&chassisModel),
	m_pLocalizer(&localizer),
	m_pCrashDetector(&crashDetector),
	m_pPathPlanner(&pathPlanner),
	m_pPathFollower(&pathFollower),
	m_pPathCoordinator(&pathCoordinator),
	m_pExplorer(&explorer)
{
}

void RobotCore::Update(const ControlSensorFrame& frame)
{
	// Update can be executed only if previous frame is known
	if (!m_PrevFrameValid)
	{
		m_PrevFrame = frame;
		m_PrevFrameValid = true;
		return;
	}

	Chassis::TOdometerCounter odoDiff = Chassis::CalculateOdometerDifference(frame.m_Odometer, m_PrevFrame.m_Odometer);

	ZoneScopedN("RobotCore::Update");

	// CrashDetector must be updated before Localizer. Localizer takes long time.
	m_pCrashDetector->Update(frame, m_PrevFrame);

	if (m_UpdateLocalizer)
	{
		Chassis::ChassisInputs inputs;
		inputs.m_MoveState = frame.m_MoveState;
		inputs.m_OdoDiff = odoDiff;
		inputs.m_Steer = frame.m_Steer;
		Localizer::Mode localizerMode = static_cast<Localizer::Mode>(m_LocalizerMode.load());
		m_pLocalizer->Update(inputs, frame.m_ScanSamples, localizerMode);
	}

	// Explorer before PathCoordinator
	m_pExplorer->Update();

	// PathCoordinator before Follower
	m_pPathCoordinator->Update();

	// Path follower and output controls values
	const Pose& meanPose = m_pLocalizer->GetMeanPose();
	m_pPathFollower->Update(meanPose, frame);

	{
		float ctrlCurvature, ctrlSpeed_mps;
		bool ctrlBrake;
		m_pPathFollower->GetOutputControls(ctrlCurvature, ctrlSpeed_mps, ctrlBrake);

		m_OutControls.m_Steer = m_pChassisModel->CalculateSteeringInput(ctrlCurvature);
		m_OutControls.m_Brake = ctrlBrake;
		m_OutControls.m_MotorSpeed_Hz = m_pChassisModel->CalculateMotorFreqInput(ctrlSpeed_mps);

		//printf("RobotCore::Update: Output controls: steer: %.3f speed: %.2f Hz\n", m_OutControls.m_Steer, m_OutControls.m_MotorSpeed_Hz);
	}

	m_PrevFrame = frame;
}

void RobotCore::ResetLocalizer()
{
	m_pLocalizer->Reset();
}

void RobotCore::SwitchLocalizerMode(Localizer::Mode newMode)
{
	m_pLocalizer->SwitchMode(newMode);
	m_LocalizerMode.store(static_cast<char>(newMode));
}

bool RobotCore::GetBrakeOverride_mt() const
{
	return m_pCrashDetector->GetBrakeOverride_mt();
}

void RobotCore::ExploreNextPosition()
{
	m_pExplorer->StartExploration();
}