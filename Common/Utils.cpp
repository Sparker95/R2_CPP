#include "Utils.h"

#include <Eigen/Dense>

void Utils::ScanSensorToMatrix(const Sensor::ScanSample* pSamples, uint scanSize, Scan::TScanPolar& outScanPolar)
{
	outScanPolar.resize(Eigen::NoChange, scanSize);
	for (uint i = 0; i < scanSize; i++)
	{
		outScanPolar(0, i) = pSamples[i].m_Angle_rad;
		outScanPolar(1, i) = pSamples[i].m_Dist_m;
	}
}
