#include "Chassis.h"

#include "Utils.h"

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
