#include "Scan.h"

#include <cmath>

// Tracy
#include "Tracy/tracy/Tracy.hpp"

#include "Pose.h"
#include "MathUtils.h"
#include "Vector2.h"
#include "CommonIntTypes.h"
#include "Utils.h"

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

void Scan::CompensateScanMotion(const std::vector<Sensor::ScanSample>& scanPolar,
	const Chassis::ChassisModel& chassisModel, const Chassis::ChassisInputs& inputs, std::vector<Sensor::ScanSample>& outScanPolarComp)
{
	ZoneScopedN("Scan::CompensateScanMotion");

	using TMoveState = Chassis::MoveState;

	float steer = inputs.m_Steer;
	TMoveState moveState = inputs.m_MoveState;

	if (moveState == Chassis::MoveState::STOP)
	{
		outScanPolarComp = scanPolar;
		return;
	}

	uint nSamples = static_cast<uint>(scanPolar.size());
	TMoveState invMoveState = moveState == TMoveState::FORWARD ? TMoveState::BACKWARD : TMoveState::FORWARD;

	const Chassis::ChassisConfig& chassisConfig = chassisModel.GetChassisConfig();
	Pose sensorOffset_LS(chassisConfig.m_sensorOffsetFwd, chassisConfig.m_sensorOffsetSide, chassisConfig.m_sensorOffsetDir_rad);

	outScanPolarComp.resize(nSamples);

	float prevSampleDir = scanPolar[0].m_Angle_rad;
	float dirAcc = 0; // Accumulator, increases from 0 to 2 * pi(ideally)
	for (uint sampleId = 0; sampleId < nSamples; sampleId++)
	{
		float prevDirDiff = -Utils::WrapAngleDiff(scanPolar[sampleId].m_Angle_rad - prevSampleDir);
		dirAcc += prevDirDiff;

		float odoExtraParam = 1.0f - dirAcc / PI_2X; // Extrapolation parameter 0..1

		// Odometer value extrapolated backwards
		float extraOdo = odoExtraParam * inputs.m_OdoDiff;

		// Extrapolated sample pose
		Chassis::ChassisInputs tempChassisInputs;
		tempChassisInputs.SetValues(extraOdo, invMoveState, steer);

		// Where chassis origin was
		Pose extraPoseDiff_LS = chassisModel.CalculatePoseDiff_LS(tempChassisInputs);

		// Convert the pose difference to Sensor Space
		Pose extraPoseDiff_SS = chassisModel.PoseDiff_LSTOSS(extraPoseDiff_LS);

		float extraSampleDir_SS = extraPoseDiff_SS.dir + scanPolar[sampleId].m_Angle_rad;
		Vector2 extraRayEndPos_SS(extraPoseDiff_SS.x + scanPolar[sampleId].m_Dist_m * std::cos(extraSampleDir_SS),
			extraPoseDiff_SS.y + scanPolar[sampleId].m_Dist_m * std::sin(extraSampleDir_SS));

		float compRayDir = Utils::WrapAngleFull(std::atan2(extraRayEndPos_SS.y, extraRayEndPos_SS.x));
		float compRayDist = extraRayEndPos_SS.Length();

		outScanPolarComp[sampleId].m_Angle_rad = compRayDir;
		outScanPolarComp[sampleId].m_Dist_m = compRayDist;

		prevSampleDir = scanPolar[sampleId].m_Angle_rad;
	}
}
