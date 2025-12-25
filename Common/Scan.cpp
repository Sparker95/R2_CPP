#include "Scan.h"
#include "MathUtils.h"

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

void Scan::ScanCartToPolar(const TScanCart& scanCart, TScanPolar& outScanPolar)
{
	uint scanSize = static_cast<uint>(scanCart.cols());
	outScanPolar.resize(Eigen::NoChange, scanSize);

	for (uint i = 0; i < scanSize; i++)
	{
		float x = scanCart(0, i);
		float y = scanCart(1, i);

		float dir = std::atan2(y, x);
		float dist = std::sqrt(x * x + y * y);

		outScanPolar(0, i) = dir;
		outScanPolar(1, i) = dist;
	}
}

void Scan::ScanPolarToCart(const TScanPolar& scanPolar, TScanCart& outScanCart, float dirOffset /*= 0.0f*/)
{
	uint scanSize = static_cast<uint>(scanPolar.cols());
	outScanCart.resize(Eigen::NoChange, scanSize);

	for (uint i = 0; i < scanSize; i++)
	{
		float dir = scanPolar(0, i);
		float dist = scanPolar(1, i);

		float _cosDir = std::cos(dir);
		float _sinDir = std::sin(dir);

		outScanCart(0, i) = dist * _cosDir;
		outScanCart(1, i) = dist * _sinDir;
	}
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
