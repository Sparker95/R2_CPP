#include "Chassis.h"

#include <cmath>

#include "Utils.h"

//------------------------------------------------------------------------------------------------------------------------------
// Chassis Statistics Model

Chassis::ChassisModel::ChassisModel(const ChassisConfig& chassisConfig, const ChassisStatsConfig& statsConfig) :
	m_ChassisConfig(chassisConfig), m_StatsConfig(statsConfig)
{
	m_Rng.seed(56418);
}

Pose Chassis::ChassisModel::CalculateRandomPoseDiff_LS(const ChassisInputs& inputs, ChassisInputs& outInputs_rnd)
{
	float odoDiff = inputs.m_OdoDiff;
	MoveState moveState = inputs.m_MoveState;
	float steerInput = inputs.m_Steer;

	// Stopped:
	if (moveState == MoveState::STOP || odoDiff == 0)
	{
		outInputs_rnd = inputs;
		return Pose(0, 0, 0);
	}

	const ChassisConfig& chassisConf = m_ChassisConfig;
	const ChassisStatsConfig& statsConf = m_StatsConfig;

	using TUniformDist = std::uniform_real_distribution<float>;

	// Calculate and randomize travel distance
	float wheelRadius_rnd = TUniformDist(
		chassisConf.m_wheelRadius - statsConf.m_WheelRadiusTol_abs,
		chassisConf.m_wheelRadius + statsConf.m_WheelRadiusTol_abs)(m_Rng);

	float wheelTurnAnglePerOdoPulse_rad = ((1.0f / static_cast<float>(chassisConf.m_encoderPulses)) *
		(static_cast<float>(chassisConf.m_gearDiffIn) / static_cast<float>(chassisConf.m_gearDiffOut)) * PI_2X);

	float odoDiff_rnd = TUniformDist(
		odoDiff - statsConf.m_OdoFreeplay_abs,
		odoDiff + statsConf.m_OdoFreeplay_abs)(m_Rng);

	float wheelTurnAngle_rad = wheelTurnAnglePerOdoPulse_rad * odoDiff_rnd +
		TUniformDist(-statsConf.m_WheelFreeplay_rad, statsConf.m_WheelFreeplay_rad)(m_Rng);

	float moveDist_abs = wheelTurnAngle_rad * wheelRadius_rnd;

	// Randomize steer input
	float steerTol_abs = statsConf.m_SteerTolTurn_abs;
	if (steerInput == 0)
		steerTol_abs = statsConf.m_SteerTolStraight_abs;
	float steerInput_rnd = TUniformDist(steerInput - steerTol_abs, steerInput + steerTol_abs)(m_Rng);

	// Return the randomized inputs values
	outInputs_rnd.SetValues(odoDiff_rnd, moveState, steerInput_rnd);

	float moveSign = 1.0f;
	float steerSign = 1.0f;
	if (moveState == MoveState::BACKWARD)
		moveSign = -1.0f;
	if (steerInput_rnd < 0.0f)
		steerSign = -1.0f;

	// Moving straight :
	if (steerInput_rnd == 0)
		return Pose(moveSign * moveDist_abs, 0, 0);

	// Turn radius of rear axle center point
	float innerWheelAngle_rad = std::fabs(steerInput_rnd) * chassisConf.m_steerSensitivity_rad;
	float innerWheelTurnRadius = chassisConf.m_wheelBase / std::tan(innerWheelAngle_rad);
	float rearAxleCenterTurnRadius = 0.5f * chassisConf.m_trackWidth + innerWheelTurnRadius;

	float moveArcAngle = moveDist_abs / rearAxleCenterTurnRadius;

	float ddir = -steerSign * moveSign * moveArcAngle; // Invesrsed because positive steering rotates clockwise
	float dx = moveSign * rearAxleCenterTurnRadius * std::sin(moveArcAngle); // Forward
	float dy = -steerSign * std::fabs(dx) * std::tan(moveArcAngle);

	return Pose(dx, dy, ddir);
}

//------------------------------------------------------------------------------------------------------------------------------
// Various functions

Pose Chassis::ChassisModel::CalculatePoseDiff_LS(const ChassisInputs& inputs) const
{
	float odoDiff = inputs.m_OdoDiff;
	Chassis::MoveState moveState = inputs.m_MoveState;
	float steerInput = inputs.m_Steer;

	// Stopped:
	if (moveState == MoveState::STOP || odoDiff == 0)
		return Pose(0, 0, 0);

	const ChassisConfig& c = m_ChassisConfig;
	float distPerOdoPulse = ((1.0f / static_cast<float>(c.m_encoderPulses)) *
		(static_cast<float>(c.m_gearDiffIn) / static_cast<float>(c.m_gearDiffOut)) * PI_2X * c.m_wheelRadius);
	float moveDistAbs = odoDiff * distPerOdoPulse;

	float moveSign = 1.0f;
	float steerSign = 1.0f;
	if (moveState == MoveState::BACKWARD)
		moveSign = -1.0f;
	if (steerInput < 0.0f)
		steerSign = -1.0f;

	// Moving straight :
	if (steerInput == 0)
		return Pose(moveSign * moveDistAbs, 0, 0);

	// Turn radius of rear axle center point
	float innerWheelAngle_rad = std::abs(steerInput) * c.m_steerSensitivity_rad;
	float innerWheelTurnRadius = c.m_wheelBase / std::tan(innerWheelAngle_rad);
	float rearAxleCenterTurnRadius = 0.5f * c.m_trackWidth + innerWheelTurnRadius;

	float moveArcAngle = moveDistAbs / rearAxleCenterTurnRadius;

	float ddir = -steerSign * moveSign * moveArcAngle; // # Invesrsed because positive steering rotates clockwise
	float dx = moveSign * rearAxleCenterTurnRadius * std::sin(moveArcAngle); // Forward
	float dy = -steerSign * std::abs(dx) * std::tan(moveArcAngle);

	return Pose(dx, dy, ddir);
}

Pose Chassis::ChassisModel::PoseDiff_LSTOSS(const Pose& poseDiff_LS) const
{
	Eigen::Matrix2f mRot = Utils::MRot(-m_ChassisConfig.m_sensorOffsetDir_rad);
	Eigen::Vector2f poseDiffXY_SS = mRot * Eigen::Vector2f(poseDiff_LS.x, poseDiff_LS.y);
	return Pose(poseDiffXY_SS(0), poseDiffXY_SS(1), poseDiff_LS.dir);
}

Pose Chassis::ChassisModel::PoseDiff_LSToSS(const Pose& poseDiff_SS) const
{
	Eigen::Matrix2f mRot = Utils::MRot(m_ChassisConfig.m_sensorOffsetDir_rad);
	Eigen::Vector2f poseDiffXY_LS = mRot * Eigen::Vector2f(poseDiff_SS.x, poseDiff_SS.y);
	return Pose(poseDiffXY_LS(0), poseDiffXY_LS(1), poseDiff_SS.dir);
}

float Chassis::ChassisModel::CalculateSteeringInput(float curvature) const
{
	// Go straight
	if (curvature == 0)
		return 0;

	const auto& c = m_ChassisConfig;

	float rearAxleCenterTurnRadius = 1.0f / std::abs(curvature);

	float innerWheelTurnRadius = rearAxleCenterTurnRadius - 0.5f * c.m_trackWidth;
	float innerWheelAngle_rad = std::atan(c.m_wheelBase / innerWheelTurnRadius);
	float steerInput = innerWheelAngle_rad / c.m_steerSensitivity_rad;
	
	return std::copysign(steerInput, curvature);
}

float Chassis::ChassisModel::CalculateMotorFreqInput(float speed_mps) const
{
	const ChassisConfig& c = m_ChassisConfig;
	float distPerOdoPulse = ((1.0f / static_cast<float>(c.m_encoderPulses)) *
		(static_cast<float>(c.m_gearDiffIn) / static_cast<float>(c.m_gearDiffOut)) * PI_2X * c.m_wheelRadius);

	float freq_Hz = speed_mps / distPerOdoPulse;
	return freq_Hz;
}

Chassis::TOdometerCounter Chassis::CalculateOdometerDifference(TOdometerCounter odoNew, TOdometerCounter odoPrev)
{
	TOdometerCounter diff = odoNew - odoPrev;
	return diff;
}
