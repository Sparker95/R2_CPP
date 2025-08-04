#pragma once

#include "Common.h"
#include "Vector2.h"

namespace Scan
{
	constexpr static uint SCAN_SIZE = 256;

	// Special value when scan ray didn't hit anything
	constexpr static float SCAN_RAY_NO_HIT_VALUE = 0.0f;

	void ScanPolarToCart(const float* scanPolar, uint scanPolarSize, float dirOffset, Vector2* outScanCart, uint& outScanCartSize);
	
	// Filters polar scan: removes all no-hit values
	void FilterScan(const float* scanPolar, uint scanPolarSize, float * outScanPolarFiltered, uint & outScanPolarFilteredSize);
}