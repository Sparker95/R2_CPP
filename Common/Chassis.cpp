#include "Chassis.h"

#include "Utils.h"

//------------------------------------------------------------------------------------------------------------------------------
// Chassis Statistics Model

Chassis::ChassisStatsModel::ChassisStatsModel(const ChassisConfig& chassisConfig, const ChassisStatsConfig& statsConfig) :
	m_ChassisConfig(chassisConfig), m_StatsConfig(statsConfig)
{
	m_Rng.seed(56418);
}

Pose Chassis::ChassisStatsModel::GetPoseDiffSample_LS(const ChassisInputs& inputs)
{
	TOdometerCounter odoDiff = inputs.m_OdoDiff;
	MoveState moveState = inputs.m_MoveState;
	float steerInput = inputs.m_Steer;

	// Stopped:
	if (moveState == MoveState::STOP || odoDiff == 0)
		return Pose(0, 0, 0);

	const ChassisConfig& chassisConf = m_ChassisConfig;
	const ChassisStatsConfig& statsConf = m_StatsConfig;

	using TUniformDist = std::uniform_real_distribution<float>;

	// Calculate and randomize travel distance
	float wheelRadius_rnd = TUniformDist(
		chassisConf.m_wheelRadius - statsConf.m_WheelRadiusTol_abs,
		chassisConf.m_wheelRadius + statsConf.m_WheelRadiusTol_abs)(m_Rng);

	float wheelTurnAnglePerOdoPulse_rad = ((1.0f / static_cast<float>(chassisConf.m_encoderPulses)) * (static_cast<float>(chassisConf.m_gearEncoder) / static_cast<float>(chassisConf.m_gearShaft)) *
		(static_cast<float>(chassisConf.m_gearDiffIn) / static_cast<float>(chassisConf.m_gearDiffOut)) * PI_2X);

	float odoDiff_rnd = TUniformDist(
		static_cast<float>(odoDiff) - statsConf.m_OdoFreeplay_abs,
		static_cast<float>(odoDiff) + statsConf.m_OdoFreeplay_abs)(m_Rng);

	float wheelTurnAngle_rad = wheelTurnAnglePerOdoPulse_rad * odoDiff_rnd +
		TUniformDist(-statsConf.m_WheelFreeplay_rad, statsConf.m_WheelFreeplay_rad)(m_Rng);

	float moveDist_abs = wheelTurnAngle_rad * wheelRadius_rnd;

	// Randomize steer input
	float steerTol_abs = statsConf.m_SteerTolTurn_abs;
	if (steerInput == 0)
		steerTol_abs = statsConf.m_SteerTolStraight_abs;
	float steerInput_rnd = TUniformDist(steerInput - steerTol_abs, steerInput + steerTol_abs)(m_Rng);

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
	float innerWheelAngle_rad = abs(steerInput_rnd) * chassisConf.m_steerSensitivity_rad;
	float innerWheelTurnRadius = chassisConf.m_wheelBase / std::tanf(innerWheelAngle_rad);
	float rearAxleCenterTurnRadius = 0.5f * chassisConf.m_trackWidth + innerWheelTurnRadius;

	float moveArcAngle = moveDist_abs / rearAxleCenterTurnRadius;

	float ddir = -steerSign * moveSign * moveArcAngle; // Invesrsed because positive steering rotates clockwise
	float dx = moveSign * rearAxleCenterTurnRadius * std::sinf(moveArcAngle); // Forward
	float dy = -steerSign * abs(dx) * std::tanf(moveArcAngle);

	return Pose(dx, dy, ddir);
}

//------------------------------------------------------------------------------------------------------------------------------
// Various functions

Pose Chassis::CalculatePoseDiff_LS(const ChassisConfig& config, TOdometerCounter odoDiff, MoveState moveState, float steerInput)
{
	// Stopped:
	if (moveState == MoveState::STOP || odoDiff == 0)
		return Pose(0, 0, 0);

	const ChassisConfig& c = config;
	float distPerOdoPulse = ((1.0f / static_cast<float>(c.m_encoderPulses)) * (static_cast<float>(c.m_gearEncoder) / static_cast<float>(c.m_gearShaft)) *
		(static_cast<float>(c.m_gearDiffIn) / static_cast<float>(c.m_gearDiffOut)) * PI_2X * c.m_wheelRadius);
	float moveDistAbs = static_cast<float>(odoDiff) * distPerOdoPulse;

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
	float innerWheelTurnRadius = c.m_wheelBase / std::tanf(innerWheelAngle_rad);
	float rearAxleCenterTurnRadius = 0.5f * c.m_trackWidth + innerWheelTurnRadius;

	float moveArcAngle = moveDistAbs / rearAxleCenterTurnRadius;

	float ddir = -steerSign * moveSign * moveArcAngle; // # Invesrsed because positive steering rotates clockwise
	float dx = moveSign * rearAxleCenterTurnRadius * std::sinf(moveArcAngle); // Forward
	float dy = -steerSign * abs(dx) * std::tanf(moveArcAngle);

	return Pose(dx, dy, ddir);
}

Pose Chassis::PoseDiff_LSTOSS(const ChassisConfig& config, const Pose& poseDiff_LS)
{
	Eigen::Matrix2f mRot = Utils::MRot(-config.m_sensorOffsetDir_rad);
	Eigen::Vector2f poseDiffXY_SS = mRot * Eigen::Vector2f(poseDiff_LS.x, poseDiff_LS.y);
	return Pose(poseDiffXY_SS(0), poseDiffXY_SS(1), poseDiff_LS.dir);
}

Pose Chassis::PoseDiff_LSToSS(const ChassisConfig& config, const Pose& poseDiff_SS)
{
	Eigen::Matrix2f mRot = Utils::MRot(config.m_sensorOffsetDir_rad);
	Eigen::Vector2f poseDiffXY_LS = mRot * Eigen::Vector2f(poseDiff_SS.x, poseDiff_SS.y);
	return Pose(poseDiffXY_LS(0), poseDiffXY_LS(1), poseDiff_SS.dir);
}

Chassis::TOdometerCounter Chassis::CalculateOdometerDifference(TOdometerCounter odoNew, TOdometerCounter odoPrev)
{
	TOdometerCounter diff = odoNew - odoPrev;
	return diff;
}
