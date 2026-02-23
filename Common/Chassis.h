#pragma once

// Code related to chassis, common on Raspberry Pi and control panel

#include <stdint.h>
#include <random>

#include "Common.h"
#include "CommonIntTypes.h"
#include "MathUtils.h"

namespace Chassis
{
	// Also used for network serialization
	enum class MoveState : uint8_t
	{
		STOP     = 0,
		FORWARD  = 1,
		BACKWARD = 2
	};

	// Same type is used on the motor controller board
	using TOdometerCounter = uint16_t;

	struct ChassisConfig
	{
		// Wheels, size of car
		float m_wheelBase = 0.26f;
		float m_trackWidth = 0.17f;
		float m_wheelRadius = 0.03125f;
		float m_width = 0.22f;
        float m_lengthFwd = 0.33f; // Relative to rear axle center point
        float m_lengthBwd = 0.04f;
		
		// Radians per unit of input, see ChassisMeasurement
		float m_steerSensitivity_rad = (PI / 180.0f) * (23.0f / 0.68f);

		// Sensor
		float m_sensorOffsetFwd = 0.185f; // Relative to rear axle center point
		float m_sensorOffsetSide = 0;
		float m_sensorOffsetDir_rad = 0.5f * PI; // Counter clock wise

		// Transmission
		// gear Motor->gear Shaft -> (gear Diff In->gear Diff Out)->wheel Radius
		//                    |
		//                    V
		//               encoder Pulses

		uint m_gearMotor = 14; // Not used
		uint m_gearShaft = 72; // Not used
		uint m_gearDiffIn = 13;
		uint m_gearDiffOut = 38;

		// Encoder - mounted on main shaft
		uint m_encoderPulses = 20; // Per revolution
	};

	struct ChassisStatsConfig
	{
		float m_WheelRadiusTol_abs = 0.01f;   // +- Meters
		float m_OdoFreeplay_abs = 0.01f;      // +- Number of pulses
		float m_WheelFreeplay_rad = 0.01f;    // +- Radians
		float m_SteerTolStraight_abs = 0.01f; // +- Steering units
		float m_SteerTolTurn_abs = 0.01f;     // +- Steering units
	};

	// Inputs for chassis motion model
	struct ChassisInputs
	{
		float m_OdoDiff = 0; // It is integer in real case, but can be float for interpolation purposes.
		MoveState m_MoveState = MoveState::STOP;
		float m_Steer = 0;

		void SetValues(float odoDiff, MoveState moveState, float steer) { m_OdoDiff = odoDiff; m_MoveState = moveState; m_Steer = steer; }
	};

	class ChassisModel
	{
	private:
		const ChassisConfig m_ChassisConfig;
		const ChassisStatsConfig m_StatsConfig;
		std::minstd_rand m_Rng;

	public:
		ChassisModel(const ChassisConfig& chassisConfig, const ChassisStatsConfig& statsConfig);

		const ChassisConfig& GetChassisConfig() const { return m_ChassisConfig; }

		// Calculates pose difference with randomization
		Pose CalculateRandomPoseDiff_LS(const ChassisInputs& inputs, ChassisInputs& outInputs_rnd);

		// Calculates ideal pose difference, without randomization
		Pose CalculatePoseDiff_LS(const ChassisInputs& inputs) const;

		// Converts pose difference from Local Space to Sensor Space
		Pose PoseDiff_LSTOSS(const Pose& poseDiff_LS) const;

		// Converts pose difference from Sensor Space to Local Space
		Pose PoseDiff_LSToSS(const Pose& poseDiff_SS) const;
	};

	//------------------------------------------------------------------------------------------------------------------------
	// Various functions

	TOdometerCounter CalculateOdometerDifference(TOdometerCounter odoNew, TOdometerCounter odoPrev);

}
