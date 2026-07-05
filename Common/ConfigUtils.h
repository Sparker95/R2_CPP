#pragma once

#include "Localizer.h"
#include "CrashDetector.h"

/*
Contains functions for reading config files of many classes.
*/

namespace Chassis { struct ChassisStatsConfig; }
namespace PathFinding { struct Config; }
struct PathFollowerConfig;
struct ExplorerConfig;

class Config;

namespace ConfigUtils
{
	bool ReadLocalizerConfig(const Config& config, Localizer::Config& outConfig);
	bool ReadChassisStatsConfig(const Config& config, Chassis::ChassisStatsConfig& outConfig);
	bool ReadPathFindingConfig(const Config& config, PathFinding::Config& outConfig);
	bool ReadCrashDetectorConfig(const Config& cfg, CrashDetector::Config& outCrashDetectorSettings);
	bool ReadPathFollowerConfig(const Config& cfg, PathFollowerConfig& outConfig);
	bool ReadExplorerConfig(const Config& cfg, ExplorerConfig& outConfig);
}