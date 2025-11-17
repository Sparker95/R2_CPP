#pragma once

namespace ChassisSettings
{
	// Configuration of controls
	struct ControlSettings
	{
		float m_fSteerOutputMax = 0.5f;
		float m_fSpeedSlow_Hz = 200.0f;
		float m_fSpeedCruise_Hz = 400.0f;
		float m_fSpeedFast_Hz = 600.0f;
	};

	struct SizeSettings
	{
		// wheel base, track, etc
	};
};