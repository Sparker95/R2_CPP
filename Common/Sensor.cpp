#include "Sensor.h"

#include "MathUtils.h"

void Sensor::ConvertSampleIntToFloat(ushort angle, ushort dist, float& outAngle_rad, float& outDist_m)
{
	// !!! Here we convert sensor spin direction. Real sensor spins clockwise,
	// while notation in rest of software is mathematical: positive is counter clockwise.

	// Actual heading = rawAngle / 64.0 deg
	outAngle_rad = PI_2X - static_cast<float>(angle) * (PI_2X / (64.0f * 360.0f));

	// Actual distance = rawDistance / 4.0 mm
	outDist_m = static_cast<float>(dist) * (1.0f / 4000.0f);
}

void Sensor::ConvertSampleIntToFloat(const Sensor::ScanSampleInt& sampleInt, Sensor::ScanSample& outSample)
{
	Sensor::ConvertSampleIntToFloat(sampleInt.m_Angle, sampleInt.m_Dist, outSample.m_Angle_rad, outSample.m_Dist_m);
}
