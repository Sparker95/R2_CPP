#pragma once

#include "Chassis.h"

class Config;

namespace ChassisUtils
{
	bool ReadStatsConfig(const Config& config, Chassis::ChassisStatsConfig& outConfig);
}