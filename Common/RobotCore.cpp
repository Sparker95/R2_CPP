#include "RobotCore.h"

#include "TracyPlug.h"

#include "Chassis.h"
#include "Localizer.h"
#include "CrashDetector.h"

RobotCore::RobotCore(Localizer& localizer, CrashDetector& crashDetector, PathPlanner& pathPlanner)
	: m_pLocalizer(&localizer), m_pCrashDetector(&crashDetector), m_pPathPlanner(&pathPlanner)
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

	if (m_UpdateLocalizer && odoDiff != 0)
	{
		Chassis::ChassisInputs inputs;
		inputs.m_MoveState = frame.m_MoveState;
		inputs.m_OdoDiff = odoDiff;
		inputs.m_Steer = frame.m_Steer;
		Localizer::Mode localizerMode = static_cast<Localizer::Mode>(m_LocalizerMode.load());
		m_pLocalizer->Update(inputs, frame.m_ScanSamples, localizerMode);
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

bool RobotCore::GetBrakeOverride() const
{
	return m_pCrashDetector->GetBrakeOverride_mt();
}