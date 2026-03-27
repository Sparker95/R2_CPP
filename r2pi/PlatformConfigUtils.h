#pragma once

class Config;

#include "ChassisInterface.h"
#include "SensorInterface.h"

namespace ChassisSettings
{
	struct ControlSettings;
}

// Functions to read config files for platform specific classes
namespace PlatformConfigUtils
{
	bool ReadChassisInterfaceSettingsConfig(const Config& cfg, ChassisInterface::AllSettings& outSettings);
	bool ReadChassisControlSettingsConfig(const Config& cfg, ChassisSettings::ControlSettings& outSettings);
	bool ReadSensorInterfaceSettingsConfig(const Config& cfg, SensorInterface::Settings& outSettings);
}