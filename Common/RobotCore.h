#pragma once

#include "ControlSensorFrame.h"

// Handles the Update() cycle. Propagates data between other parts.

class Localizer;

class RobotCore
{
private:
	Localizer* m_pLocalizer;
	ControlSensorFrame m_PrevFrame;
	bool m_PrevFrameValid = false;

	// Switches for functionality
	bool m_UpdateLocalizer = true;

public:
	RobotCore(Localizer& pLocalizer);

	void Update(const ControlSensorFrame& frame);

	void SetUpdateLocalizer(bool value) { m_UpdateLocalizer = value; }
};