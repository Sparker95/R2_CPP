#pragma once

#include "Vector2.h"
#include "Pose.h"

#include <vector>

struct ControlSensorFrame;

struct PathFollowerConfig
{
	float m_CompletionRadius = 0.06f;
	float m_LookaheadDistance = 0.4f; // !!! Must be smaller than 2*TurnRadius
	float m_BrakeDist = 0.2f;
	float m_Speed_mps = 0.1f;
};

class PathFollower
{
public:
	using uint = unsigned int;

	struct PathPoint
	{
		enum class Type : unsigned char
		{
			MAIN = 0,    // Point from pathfinding result
			MAIN_END,    // Main point, also end of a section or whole path
			MAIN_SMOOTH, // Smoothed points between MAIN(_END) points
			EXTENSION    // Extensions beyond end point, to aid steering
		};

		Vector2 m_Pos = {};
		Vector2 m_Dir = {}; // !!! Only set for MAIN and MAIN_END points, not needed for others.
		float m_DistFromStart;
		Type m_Type = Type::MAIN;
		bool m_OutMoveFwd = false; // True - move forward from this to next node

		/*
		Point types:
		  Name       | Can be end of section?
		- Original     Yes
		- Smooth       No
		- Extension    No

		Flags
		- End of section

		Point types:
		- MAIN
		- MAIN_END
		- SMOOTH
		- EXTENSION
		*/
	};

	enum class State : unsigned char
	{
		IDLE,
		MOVING,   // Moving
		STOPPING, // Braking and waiting for stop
		ERROR
	};

	PathFollower(const PathFollowerConfig& config);

	// Main external interface
	void Update(const Pose& pose, const ControlSensorFrame& frame);
	void FollowPath(const std::vector<PathPoint>& path);
	void ResetAndStop();

	// Getters
	void GetOutputControls(float& outCurvature, float& outSpeed_mps, bool& outBrake) const
	{
		outCurvature = m_CtrlCurvature;
		outSpeed_mps = m_CtrlSpeed_mps;
		outBrake = m_CtrlBrake;
	}

	State GetState() const                        { return m_State; }
	const Vector2& GetLookaheadPos() const        { return m_LookaheadPos; }
	const std::vector<PathPoint>& GetPath() const { return m_Path; }
	uint GetCurrentPointId() const                { return m_CurrentPointId; }
	uint GetCurrentEndPointId() const             { return m_CurrentEndPointId; }

private:
	std::vector<PathPoint> m_Path;
	const PathFollowerConfig m_Config;

	// State of path following
	Vector2 m_LookaheadPos;
	uint m_CurrentPointId = static_cast<uint>(-1);
	uint m_CurrentEndPointId = static_cast<uint>(-1); // End point of current segment
	State m_State = State::IDLE;

	// Outputs for car control
	float m_CtrlCurvature = 0;
	float m_CtrlSpeed_mps = 0;
	bool m_CtrlBrake = false;

	void UpdateCurrentPathPoint(const Pose& pose, const Vector2& dir);
	void UpdateSteeringSolution(const Pose& pose, const Vector2& dir);
	void UpdateSpeedSolution(const Pose& pose, const Vector2& dir);
};