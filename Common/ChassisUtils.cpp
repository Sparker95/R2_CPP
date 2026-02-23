#include "Config.h"
#include "ChassisUtils.h"
#include "MathUtils.h"

bool ChassisUtils::ReadStatsConfig(const Config& config, Chassis::ChassisStatsConfig& outConfig)
{
	Chassis::ChassisStatsConfig tempConfig;

	/*
	float m_WheelRadiusTol_abs = 0.0045f;            // +- Meters
		float m_OdoFreeplay_abs = 1.0f;                  // +- Number of pulses
		float m_WheelFreeplay_rad = 60.0f * PI / 180.0f; // +- Radians
		float m_SteerTolStraight_abs = 0.25f;            // +- Steering units
		float m_SteerTolTurn_abs = 0.5f;                 // +- Steering units
	
	*/

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
