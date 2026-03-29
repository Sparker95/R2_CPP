#include "CrashDetector.h"

#include <cmath>

#include "TracyPlug.h"

#include "ControlSensorFrame.h"
#include "Chassis.h"
#include "Utils.h"

CrashDetector::CrashDetector(const Config& config, const Chassis::ChassisModel& chassisModel) :
	m_Config(config),
	m_pChassisModel(&chassisModel)
{
}

void CrashDetector::Update(const ControlSensorFrame& frame, const ControlSensorFrame& prevFrame)
{
	if (!m_Config.m_Enabled)
		return;

	ZoneScopedN("CrashDetector::Update");

	float dt_s = 1e-3f * static_cast<float>((frame.m_TimeScanEnd_ms - prevFrame.m_TimeScanEnd_ms));

	// Current frequency of odometer
	Chassis::TOdometerCounter odoDiff = Chassis::CalculateOdometerDifference(frame.m_Odometer, prevFrame.m_Odometer);

	if (odoDiff == 0 && frame.m_MoveState == Chassis::MoveState::STOP)
	{
		// We are stopped, nothing to do
		m_BrakeOverride.store(false);
		return;
	}

	float odoFreq_Hz = static_cast<float>(odoDiff) / dt_s;

	float timeToBrake_s = odoFreq_Hz / m_Config.m_BrakeAccel_freq_s2;
	float extrapolateTime_s = timeToBrake_s + dt_s + m_Config.m_ExtraTimeToCrash_s;

	// How many pulses we are going to extrapolate in future. This is equivalent to calculating distance of extrapolated movement.
	Chassis::TOdometerCounter odoExtrapolatePulses = static_cast<Chassis::TOdometerCounter>(extrapolateTime_s * odoFreq_Hz);

	// Assume car moves certain distance, in same direction (moveState) and with same steering
	Chassis::ChassisInputs chInputs;
	chInputs.m_MoveState = frame.m_MoveState;
	chInputs.m_OdoDiff = odoExtrapolatePulses;
	chInputs.m_Steer = frame.m_Steer;
	Pose extChassisPoseDiff_LS = m_pChassisModel->CalculatePoseDiff_LS(chInputs);

	const Chassis::ChassisConfig& chassisConfig = m_pChassisModel->GetChassisConfig();
	
	Pose sensorPoseDiff_LS(chassisConfig.m_sensorOffsetFwd, chassisConfig.m_sensorOffsetSide, chassisConfig.m_sensorOffsetDir_rad);
	
	// Calculate in Local Space where will be sensor samples.
	// As chassis moves one direction, the sample positions move the opposite.
	Pose extSensorPoseDiff_LS(sensorPoseDiff_LS.x - extChassisPoseDiff_LS.x,
		sensorPoseDiff_LS.y - extChassisPoseDiff_LS.y,
		sensorPoseDiff_LS.dir - extChassisPoseDiff_LS.dir);

	// Iterate all samples. Compute their position. Check if sample is in car's box.
	bool crashDetected = false;
	float halfWidth = 0.5f * chassisConfig.m_width;
	for (const Sensor::ScanSample& sample : frame.m_ScanSamples)
	{
		float sampleDir_ls = sample.m_Angle_rad + extSensorPoseDiff_LS.dir;
		float sx_ls = extSensorPoseDiff_LS.x + std::cos(sampleDir_ls) * sample.m_Dist_m; // sample x, sample y
		float sy_ls = extSensorPoseDiff_LS.y + std::sin(sampleDir_ls) * sample.m_Dist_m;

		if (sy_ls > -halfWidth && sy_ls < halfWidth &&
			sx_ls > -chassisConfig.m_lengthBwd && sx_ls < chassisConfig.m_lengthFwd)
		{
			crashDetected = true;
			break;
		}
	}

	if (crashDetected)
	{
		// Only resets once braking is done!
		m_BrakeOverride.store(true);
	}
}

bool CrashDetector::GetBrakeOverride_mt() const
{
	return m_BrakeOverride.load();
}