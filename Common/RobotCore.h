#pragma once

#include "ControlSensorFrame.h"
#include "Localizer.h"

#include <atomic>

// Handles the Update() cycle. Propagates data between other parts.

class Localizer;
class CrashDetector;
class PathPlanner;
class PathFollower;
namespace Chassis
{
	class ChassisModel;
}

class RobotCore
{
public:
	// Control values to the outside
	struct OutChassisControls
	{
		float m_Steer = 0; // Positive - turn right
		float m_MotorSpeedNorm = 0;
		bool m_Brake = false;
	};

private:
	const Chassis::ChassisModel* m_pChassisModel;
	Localizer* m_pLocalizer;
	CrashDetector* m_pCrashDetector;
	PathPlanner* m_pPathPlanner;
	PathFollower* m_pPathFollower;

	ControlSensorFrame m_PrevFrame;
	bool m_PrevFrameValid = false;

	// Switches for functionality
	std::atomic<bool> m_UpdateLocalizer = false;
	std::atomic_char m_LocalizerMode = static_cast<char>(Localizer::Mode::LOC_AND_MAP);
	std::atomic<bool> m_AutoControls = false;

	OutChassisControls m_OutControls;
	
public:
	RobotCore(const Chassis::ChassisModel& chassisModel, Localizer& localizer, CrashDetector& crashDetector,
		PathPlanner& pathPlanner, PathFollower& pathFollower);

	void Update(const ControlSensorFrame& frame);

	void SetUpdateLocalizer(bool value) { m_UpdateLocalizer.store(value); }
	bool GetUpdateLocalizer_mt() const { return m_UpdateLocalizer.load(); }

	// False - sending remote controls to chassis, true - sending own autonomous controls.
	void SetAutoControlsMode(bool value) { m_AutoControls.store(value);}
	bool GetAutoControlsMode_mt() const { return m_AutoControls.load(); }

	void SwitchLocalizerMode(Localizer::Mode newMode);
	Localizer::Mode GetLocalizerMode_mt() const { return static_cast<Localizer::Mode>(m_LocalizerMode.load()); }

	const OutChassisControls& GetOutControls() const { return m_OutControls; }

	void ResetLocalizer();

	// Trivial getters
	const Chassis::ChassisModel* GetChassisModel() { return m_pChassisModel; }
	Localizer* GetLocalizer() { return m_pLocalizer; }
	PathPlanner* GetPathPlanner() { return m_pPathPlanner; }
	PathFollower* GetPathFollower() { return m_pPathFollower; }

	bool GetBrakeOverride_mt() const;
};