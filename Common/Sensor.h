#pragma once

// Data types used by sensor

namespace Sensor
{
	constexpr unsigned int SCAN_N_SAMPLES_MAX = 1024;

	// One sample of scan
	struct ScanSample
	{
		float m_Angle_rad;
		float m_Dist_m;
	};
}