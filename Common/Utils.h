#pragma once

#include "Pose.h"
#include "Sensor.h"
#include "Scan.h"

#include <Eigen/Core>

namespace Utils
{
	// Angle utils
	// Returns angle within 0..2*pi
	float WrapAngleFull(float angle);

	// Returns angle within -pi..pi
	float WrapAngleDiff(float angle);

	// Rotation matrix
	Eigen::Matrix2f MRot(float angle);
	
	// Converts scan from sensor's format to matrix format
	void ScanSensorToMatrix(const Sensor::ScanSample* pSamples, uint scanSize, Scan::TScanPolar& outScanPolar);

	// Adds a LS pose diff to WS pose
	Pose AddPoseDiff(const Pose& pose_WS, const Pose& poseDiff_LS);
}
