#pragma once

#include <stdint.h>
#include <vector>

#include "Chassis.h"
#include "Sensor.h"

// Handling of sensor stream blocks.
// Originally made to read sensor stream data from files, but potentially this can be used at run time as well.
// This will keep code unified between online and offline data processing. 

// One 'block' of data.
// It partially mirrors ControlPanelProtocol::SensorDataHeader
struct SensorStreamBlock
{
	uint32_t m_Timestamp_ms;
	Chassis::TOdometerCounter m_Odometer;
	Chassis::MoveState m_MoveState;
	float m_Steer;
	std::vector<Sensor::ScanSample> m_ScanSamples;
};

