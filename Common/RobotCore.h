#pragma once

#include "ControlSensorFrame.h"

#include <atomic>

// Handles the Update() cycle. Propagates data between other parts.

class Localizer;

class RobotCore
{
private:
	Localizer* m_pLocalizer;
	ControlSensorFrame m_PrevFrame;
	bool m_PrevFrameValid = false;

	// Switches for functionality
	std::atomic<bool> m_UpdateLocalizer = false;

public:
	RobotCore(Localizer& pLocalizer);

	void Update(const ControlSensorFrame& frame);

	void SetUpdateLocalizer(bool value) { m_UpdateLocalizer.store(value); }
	bool GetUpdateLocalizer() const { return m_UpdateLocalizer.load(); }

	void ResetLocalizer();

	// Trivial getters
	Localizer* GetLocalizer() { return m_pLocalizer; }
};