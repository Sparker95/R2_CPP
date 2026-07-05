#pragma once

#include "Chassis.h"

#include <atomic>

struct ControlSensorFrame;

class CrashDetector
{
public:
	struct Config
	{
		bool m_Enabled = true;

		// Brake deceleration, measured in odometer frequency per s^2, like in ChassisInterface
		float m_BrakeAccel_freq_s2 = 1.0f;

		// Extra safety margin, to factor in delays, extra space, etc
		float m_ExtraTimeToCrash_s = 0.2f;
	};

	CrashDetector(const Config& config, const Chassis::ChassisModel& chassisModel);
	void Update(const ControlSensorFrame& frame, const ControlSensorFrame& prevFrame);

	// Return true if impeding crash was detected and we must brake now.
	// It resets only when braking is completed.
	bool GetBrakeOverride_mt() const;

private:
	const Config m_Config;
	const Chassis::ChassisModel* m_pChassisModel = nullptr;
	std::atomic_bool m_BrakeOverride = false;
};