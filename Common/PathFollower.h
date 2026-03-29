#pragma once

#include "Vector2.h"
#include "Pose.h"

#include <vector>

struct PathFollowerConfig
{
	float m_CompletionRadius = 0.06f;
	float m_LookaheadDistance = 0.4f; // !!! Must be smaller than 2*TurnRadius
};

class PathFollower
{
public:
	using uint = unsigned int;

	struct PathPoint
	{
		enum class Type : unsigned char
		{
			MAIN = 0, // Normal path points
			END,      // End of a section or whole path
			EXTENSION // Extensions beyond end point, to aid steering
		};

		Vector2 m_Pos = {};
		float m_Speed = 0;
		Type m_Type = Type::MAIN;
		bool m_OutMoveFwd = false; // True - move forward from this to next node
	};

	enum class State : unsigned char
	{
		IDLE,
		RUNNING,
		END,
		ERROR
	};

	PathFollower(const PathFollowerConfig& config);

	// Main external interface
	void Update(const Pose& pose);
	void FollowPath(const std::vector<PathPoint>& path);
	void Reset();

	// Getters
	void GetOutputControls(float& outCurvature, float& outSpeed, bool& outBrake) const
	{
		outCurvature = m_CtrlCurvature;
		outSpeed = m_CtrlSpeed;
		outBrake = m_CtrlBrake;
	}

	State GetState() const                       { return m_State; }
	bool GetStateRunning() const                 { return m_State == State::RUNNING; }
	const Vector2& GetLookaheadPos() const       { return m_LookaheadPos; }
	const std::vector<PathPoint> GetPath() const { return m_Path; }
	uint GetCurrentPointId() const               { return m_CurrentPointId; }
	uint GetCurrentEndPointId() const            { return m_CurrentEndPointId; }

private:
	std::vector<PathPoint> m_Path;
	PathFollowerConfig m_Config;

	// State of path following
	Vector2 m_LookaheadPos;
	uint m_CurrentPointId = static_cast<uint>(-1);
	uint m_CurrentEndPointId = static_cast<uint>(-1); // End point of current segment
	State m_State = State::IDLE;

	// Outputs for car control
	float m_CtrlCurvature = 0;
	float m_CtrlSpeed = 0;
	bool m_CtrlBrake = false;
};