#pragma once

// Code related to chassis, common on Raspberry Pi and control panel

#include <stdint.h>

#include "Common.h"
#include "IntTypes.h"
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
		float m_trackWidth = 0.16f;
		float m_wheelRadius = 0.065f / 2.0f;
		float m_width = 0.22f;
		float m_length = 0.38f;
		
		// Radians per unit of input, see ChassisMeasurement
		float m_steerSensitivity_rad = (PI / 180.0f) * (28.5f / 0.68f);

		// Sensor
		float m_sensorOffsetFwd = 0.185f; // Relative to rear axle center point
		float m_sensorOffsetSide = 0;
		float m_sensorOffsetDir_rad = 0.5f * PI; // Counter clock wise

		// Transmission
		// gear Motor->gear Shaft -> (gear Diff In->gear Diff Out)->wheel Radius
		//                    |
		//                    V
		//               gear Encoder->encoder Pulses

		uint m_gearMotor = 14; // # Not needed
		uint m_gearShaft = 72;
		uint m_gearDiffIn = 13;
		uint m_gearDiffOut = 38;

		// Encoder
		uint m_gearEncoder = 10;
		uint m_encoderPulses = 8; // Per revolution
	};

	Pose CalculatePoseDiff_LS(const ChassisConfig& config, TOdometerCounter odoDiff, MoveState moveState, float steerInput);
	
	// Converts pose difference from Local Space to Sensor Space
	Pose PoseDiff_LSTOSS(const ChassisConfig& config, const Pose& poseDiff_LS);

	// Converts pose difference from Sensor Space to Local Space
	Pose PoseDiff_LSToSS(const ChassisConfig& config, const Pose& poseDiff_SS);

	TOdometerCounter CalculateOdometerDifference(TOdometerCounter odoNew, TOdometerCounter odoPrev);

}