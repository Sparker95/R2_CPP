#pragma once

#include "Common.h"
#include "Sensor.h"
#include "Scan.h"

namespace Utils
{
	// Converts scan from sensor's format to matrix format
	void ScanSensorToMatrix(const Sensor::ScanSample* pSamples, uint scanSize, Scan::TScanPolar& outScanPolar);
}