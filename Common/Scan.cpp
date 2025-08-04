#include "Scan.h"

#include <cmath>

void Scan::ScanPolarToCart(const float* scanPolar, uint scanPolarSize, float dirOffset, Vector2* outScanCart, uint& outScanCartSize)
{
	uint scanCartSize = 0;
	float fScanPolarSize = static_cast<float>(scanPolarSize);
	for (uint i = 0; i < scanPolarSize; i++)
	{
		float dist = scanPolar[i];
		
		if (dist == SCAN_RAY_NO_HIT_VALUE)
			continue;

		float dir = dirOffset + static_cast<float>(i) * PI_2X / fScanPolarSize;
		float _cosDir = std::cos(dir);
		float _sinDir = std::sin(dir);

		Vector2& vOut = outScanCart[scanCartSize];
		vOut.x = dist * _cosDir;
		vOut.y = dist * _sinDir;
		scanCartSize++;
	}

	outScanCartSize = scanCartSize;
}

void Scan::FilterScan(const float* scanPolar, uint scanPolarSize, float* outScanPolarFiltered, uint& outScanPolarFilteredSize)
{
	uint scanPolarFilteredSize = 0;
	for (uint i = 0; i < scanPolarSize; i++)
	{
		float dist = scanPolar[i];
		if (dist == SCAN_RAY_NO_HIT_VALUE)
			continue;

		outScanPolarFiltered[scanPolarFilteredSize] = dist;
		scanPolarFilteredSize++;
	}

	outScanPolarFilteredSize = scanPolarFilteredSize;
}
