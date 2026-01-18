#include "RobotCore.h"

#ifdef TRACY_ENABLE
#include "Tracy/tracy/Tracy.hpp"
#else
#include "TracyPlug.h"
#endif

#include "Chassis.h"
#include "Localizer.h"

RobotCore::RobotCore(Localizer& pLocalizer)
	: m_pLocalizer(&pLocalizer)
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

	if (odoDiff == 0)
	{
		m_PrevFrame = frame;
		return;
	}

	if (m_UpdateLocalizer)
	{
		Chassis::ChassisInputs inputs;
		inputs.m_MoveState = frame.m_MoveState;
		inputs.m_OdoDiff = odoDiff;
		inputs.m_Steer = frame.m_Steer;
		m_pLocalizer->Update(inputs, frame.m_ScanSamples);
	}

	m_PrevFrame = frame;
}

void RobotCore::ResetLocalizer()
{
	m_pLocalizer->Reset();
}
