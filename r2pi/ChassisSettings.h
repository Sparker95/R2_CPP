#pragma once

namespace ChassisSettings
{
	// Configuration of controls
	struct ControlSettings
	{
		float m_SteerOutputMax = 0.5f;
		float m_SpeedSlow_Hz = 200.0f;
		float m_SpeedCruise_Hz = 400.0f;
		float m_SpeedFast_Hz = 600.0f;
	};

	struct SizeSettings
	{
		// wheel base, track, etc
	};
};