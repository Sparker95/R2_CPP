#include "ConfigUtils.h"

#include "Config.h"
#include "Localizer.h"
#include "Chassis.h"
#include "MathUtils.h"
#include "ProbUtils.h"
#include "PathFinding.h"
#include "CrashDetector.h"
#include "PathFollower.h"
#include "Explorer.h"

bool ConfigUtils::ReadChassisStatsConfig(const Config& config, Chassis::ChassisStatsConfig& outConfig)
{
	Chassis::ChassisStatsConfig tempConfig;

	/*
	float m_WheelRadiusTol_abs = 0.0045f;            // +- Meters
		float m_OdoFreeplay_abs = 1.0f;                  // +- Number of pulses
		float m_WheelFreeplay_rad = 60.0f * PI / 180.0f; // +- Radians
		float m_SteerTolStraight_abs = 0.25f;            // +- Steering units
		float m_SteerTolTurn_abs = 0.5f;                 // +- Steering units

	*/

	config.SetSection("ChassisStats");

	float wheelFreeplay_deg;
	bool allValuesFound = config.Get("WheelRadiusTol_abs", tempConfig.m_WheelRadiusTol_abs) &&
		config.Get("OdoFreeplay_abs", tempConfig.m_OdoFreeplay_abs) &&
		config.Get("WheelFreeplay_deg", wheelFreeplay_deg) &&
		config.Get("SteerTolStraight_abs", tempConfig.m_SteerTolStraight_abs) &&
		config.Get("SteerTolTurn_abs", tempConfig.m_SteerTolTurn_abs);

	if (!allValuesFound)
		return false;

	tempConfig.m_WheelFreeplay_rad = PI * wheelFreeplay_deg / 180.0f;
	outConfig = tempConfig;

	return true;
}

bool ConfigUtils::ReadLocalizerConfig(const Config& config, Localizer::Config& outConfig)
{
	Localizer::Config tempConfig;

	config.SetSection("Localizer");

	float l2Occupied, l2Free;
	float l2Occupied_RT, l2Free_RT; // For the Real Time map
	bool allValuesFound = config.Get("SensorPdfHalfWidth", tempConfig.m_SensorPdfHalfWidth) &&
		config.Get("SensorPdfFloor", tempConfig.m_SensorPdfFloor) &&
		config.Get("SensorScanRange", tempConfig.m_SensorScanRange) &&
		config.Get("MapResolution_m", tempConfig.m_MapRes_w) &&
		config.Get("NumParticles", tempConfig.m_nParticles) &&
		config.Get("L2OccupiedCell", l2Occupied) &&
		config.Get("L2FreeCell", l2Free) &&
		config.Get("L2OccupiedCell_RT", l2Occupied_RT) &&
		config.Get("L2FreeCell_RT", l2Free_RT) &&
		config.Get("ScanDecimation", tempConfig.m_ScanDecimation);

	if (!allValuesFound)
		return false;

	tempConfig.m_L2IntOccupiedCell = Utils::L2FloatToL2Int(l2Occupied);
	tempConfig.m_L2IntFreeCell = Utils::L2FloatToL2Int(l2Free);
	tempConfig.m_L2IntOccupiedCell_RT = Utils::L2FloatToL2Int(l2Occupied_RT);
	tempConfig.m_L2IntFreeCell_RT = Utils::L2FloatToL2Int(l2Free_RT);

	outConfig = tempConfig;

	return true;
}

bool ConfigUtils::ReadPathFindingConfig(const Config& config, PathFinding::Config& outConfig)
{
	PathFinding::Config tempConfig;

	config.SetSection("PathFinding");

	bool allValuesFound =
		config.Get("TurnRadius",        tempConfig.m_TurnRadius) &&
		config.Get("NumDirections",     tempConfig.m_NumDirs) &&
		config.Get("CarWidth",          tempConfig.m_CarWidth) &&
		config.Get("CarLengthFwd",      tempConfig.m_CarLengthFwd) &&
		config.Get("CarLengthBwd",      tempConfig.m_CarLengthBwd) &&
		config.Get("CarScaleNormal",    tempConfig.m_CarScaleNormal) &&
		config.Get("CarScaleExtended",  tempConfig.m_CarScaleExtended) &&
		config.Get("HeuristicMult",     tempConfig.m_HeuristicMult) &&
		config.Get("OffStructureMult",  tempConfig.m_OffStructureMult) &&
		config.Get("UnknownCellsMult",  tempConfig.m_HitUnknownCellsMult) &&
		config.Get("FwdBwdChangeMult",  tempConfig.m_FwdBwdChangeMult) &&
		config.Get("StructureMapWidth", tempConfig.m_StructureMapWidth) &&
		config.Get("MaxNumNodes",       tempConfig.m_MaxNodes) &&
		config.Get("MultFwd",           tempConfig.m_MultFwd) &&
		config.Get("MultTurnFwd",       tempConfig.m_MultTurnFwd) &&
		config.Get("MultBwd",           tempConfig.m_MultBwd) &&
		config.Get("MultTurnBwd",       tempConfig.m_MultTurnBwd);

	if (!allValuesFound)
		return false;

	tempConfig.Init(); // Remember to call this!
	outConfig = tempConfig;
	return true;
}

bool ConfigUtils::ReadCrashDetectorConfig(const Config& cfg, CrashDetector::Config& outCrashDetectorSettings)
{
	CrashDetector::Config s;

	cfg.SetSection("CrashDetector");

	bool cfgSuccess =
		cfg.Get("Enabled", s.m_Enabled) &&
		cfg.Get("BrakeAccel_Hz/s^2", s.m_BrakeAccel_freq_s2) &&
		cfg.Get("ExtraTimeToCrash_s", s.m_ExtraTimeToCrash_s);

	if (!cfgSuccess)
		return false;

	outCrashDetectorSettings = s;
	return true;
}

bool ConfigUtils::ReadPathFollowerConfig(const Config& cfg, PathFollowerConfig& outConfig)
{
	PathFollowerConfig tempConfig;

	cfg.SetSection("PathFollower");

	bool cfgSuccess = cfg.Get("CompletionRadius", tempConfig.m_CompletionRadius) &&
		cfg.Get("LookaheadDistance", tempConfig.m_LookaheadDistance) &&
		cfg.Get("BrakeDistance", tempConfig.m_BrakeDist) &&
		cfg.Get("Speed_m/s", tempConfig.m_Speed_mps);

	if (!cfgSuccess)
		return false;

	outConfig = tempConfig;
	return true;
}

bool ConfigUtils::ReadExplorerConfig(const Config& cfg, ExplorerConfig& outConfig)
{
	ExplorerConfig tempConfig;

	cfg.SetSection("Explorer");

	bool cfgSuccess = cfg.Get("DistMapThreshold", tempConfig.m_DistMapThreshold) &&
		cfg.Get("MoveRadius", tempConfig.m_MoveRadius) &&
		cfg.Get("MinDistToCell", tempConfig.m_MinDistToCell);

	if (!cfgSuccess)
		return false;

	outConfig = tempConfig;
	return true;
}