#pragma once

namespace ChassisSettings
{
	// Configuration of controls
	struct ControlSettings
	{
		float m_SteerOutputMax = 0.5f;
		float m_SpeedSlow_mps = 0.1f; // meters per second
		float m_SpeedCruise_mps = 0.1f;
		float m_SpeedFast_mps = 0.1f;
	};
};