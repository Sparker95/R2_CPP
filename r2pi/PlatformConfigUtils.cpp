#include "PlatformConfigUtils.h"

#include "../Common/Config.h"
#include "../Common/CrashDetector.h"

#include "ChassisInterface.h"
#include "ChassisSettings.h"
#include "SensorInterface.h"

#include <stdio.h>

bool PlatformConfigUtils::ReadChassisInterfaceSettingsConfig(const Config& cfg, ChassisInterface::AllSettings& outSettings)
{
	ChassisInterface::AllSettings s;

	bool cfgSuccess =
		// Connection settings
		cfg.Get("SerialDevPath", s.m_ConnSettings.m_SerialDevPath) &&
		// PID settings
		cfg.Get("MotorControllerKP", s.m_PidSettings.m_KP) &&
		cfg.Get("MotorControllerKI", s.m_PidSettings.m_KI) &&
		cfg.Get("MotorControllerKD", s.m_PidSettings.m_KD) &&
		cfg.Get("MotorControllerErrorScale", s.m_PidSettings.m_ErrorScale) &&
		cfg.Get("MotorControllerErrorIntegralMax", s.m_PidSettings.m_ErrorIntegralMax) &&
		// Acceleration settings
		cfg.Get("MotorControllerAccelMove", s.m_AccelSettings.m_AccelMove) &&
		cfg.Get("MotorControllerAccelBrake", s.m_AccelSettings.m_AccelBrake) &&
		// Steering settings
		cfg.Get("SteerSpeed", s.m_SteeringSettings.m_SteeringSpeed);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read chassis interface settings from config\n");
		return false;
	}

	outSettings = s;
	return true;
}

bool PlatformConfigUtils::ReadChassisControlSettingsConfig(const Config& cfg, ChassisSettings::ControlSettings& outSettings)
{
	ChassisSettings::ControlSettings s;

	bool cfgSuccess =
		cfg.Get("SteerOutputMax", s.m_SteerOutputMax) &&
		cfg.Get("SpeedSlow_Hz", s.m_SpeedSlow_Hz) &&
		cfg.Get("SpeedCruise_Hz", s.m_SpeedCruise_Hz) &&
		cfg.Get("SpeedFast_Hz", s.m_SpeedFast_Hz);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read chassis control settings from config\n");
		return false;
	}

	outSettings = s;
	return true;
}

bool PlatformConfigUtils::ReadSensorInterfaceSettingsConfig(const Config& cfg, SensorInterface::Settings& outSettings)
{
	SensorInterface::Settings s;

	bool cfgSuccess = cfg.Get("SerialDevPath", s.m_SerialDevPath) &&
		cfg.Get("MotorSpeed_Hz", s.m_MotorSpeed_Hz) &&
		cfg.Get("MotorControllerKP", s.m_MotorControllerPidSettings.m_fKP) &&
		cfg.Get("MotorControllerKI", s.m_MotorControllerPidSettings.m_fKI) &&
		cfg.Get("MotorControllerKD", s.m_MotorControllerPidSettings.m_fKD) &&
		cfg.Get("MotorControllerErrorScale", s.m_MotorControllerPidSettings.m_fErrorScale) &&
		cfg.Get("MotorControllerErrorIntegralMax", s.m_MotorControllerPidSettings.m_fErrorIntegralMax) &&
		cfg.Get("MotorControllerStartPwmValue_01", s.m_MotorControllerStartPwmValue_01) &&
		cfg.Get("MotorControllerMinPwmValue_01", s.m_MotorControllerMinPwmValue_01);

	if (!cfgSuccess)
	{
		fprintf(stderr, "Error: Failed to read sensor settings from config\n");
		return false;
	}

	outSettings = s;
	return true;
}