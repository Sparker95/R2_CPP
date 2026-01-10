#pragma once

#include "CommonIntTypes.h"

// Data types and functions used by sensor

namespace Sensor
{
	// Estimate of buffer size for initialization of arrays
	constexpr unsigned int DEFAULT_SAMPLE_BUFFER_SIZE = 256;

	// One sample of scan, used for computations
	struct ScanSample
	{
		float m_Angle_rad;
		float m_Dist_m;
	};

	// One sample of scan, same format as RawScanSample, except quality is missing.
	struct ScanSampleInt
	{
		ushort m_Angle;
		ushort m_Dist;
	};

	void ConvertSampleIntToFloat(ushort angle, ushort dist, float& outAngle_rad, float& outDist_m);
	void ConvertSampleIntToFloat(const ScanSampleInt& sampleInt, ScanSample& outSample);
}