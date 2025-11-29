#pragma once

// Code related to chassis, common on Raspberry Pi and control panel

#include <stdint.h>

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
}