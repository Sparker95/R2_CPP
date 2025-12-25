#pragma once

#include "Common.h"
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

	// Utils for Log2 probability ratio operations
	constexpr unsigned int LOG2_PROB_QUANTIZATION = 8;
	float ProbToL2Float(float prob);
	float L2FloatToProb(float l2_float);
	float L2IntToL2Float(int l2_int);
	int L2FloatToL2Int(float l2_float);
}