#pragma once

#include "ControlSensorFrame.h"
#include "Localizer.h"

#include <atomic>

// Handles the Update() cycle. Propagates data between other parts.

class Localizer;
class CrashDetector;
class PathPlanner;

class RobotCore
{
private:
	Localizer* m_pLocalizer;
	CrashDetector* m_pCrashDetector;
	PathPlanner* m_pPathPlanner;

	ControlSensorFrame m_PrevFrame;
	bool m_PrevFrameValid = false;

	// Switches for functionality
	std::atomic<bool> m_UpdateLocalizer = false;
	std::atomic_char m_LocalizerMode = static_cast<char>(Localizer::Mode::LOC_AND_MAP);

public:
	RobotCore(Localizer& localizer, CrashDetector& crashDetector, PathPlanner& pathPlanner);

	void Update(const ControlSensorFrame& frame);

	void SetUpdateLocalizer(bool value) { m_UpdateLocalizer.store(value); }
	bool GetUpdateLocalizer_mt() const { return m_UpdateLocalizer.load(); }

	void SwitchLocalizerMode(Localizer::Mode newMode);
	Localizer::Mode GetLocalizerMode_mt() const { return static_cast<Localizer::Mode>(m_LocalizerMode.load()); }

	void ResetLocalizer();

	// Trivial getters
	Localizer* GetLocalizer() { return m_pLocalizer; }
	PathPlanner* GetPathPlanner() { return m_pPathPlanner; }

	bool GetBrakeOverride() const;
};