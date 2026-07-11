#pragma once

#include "Vector2.h"
#include "PathFollower.h"
#include "ControlSensorFrame.h"

#include <functional>

class PathPlanner;
class Localizer;

// Coordinates path follower and planner. Also verifies that path is correct.
class PathCoordinator
{
public:
	using TOnPathPlanned = std::function<void(const std::vector<PathFollower::PathPoint>&)>;

private:
	enum class State
	{
		IDLE = 0,
		MOVING,            // Moving to requested destination
		MOVING_UNOBSTRUCT, // Moving on a path to leave obstruction
		PATH_OBSTR,
		FAIL
	};
	static const char* s_aStateText[5];

	const Localizer* m_pLocalizer;
	PathPlanner* m_pPathPlanner;
	PathFollower* m_pPathFollower;

	Vector2 m_MoveToPos;
	float m_MoveToPosRadius = 0; // When not 0, means that external command is valid.

	State m_State = State::IDLE;
	std::vector<TOnPathPlanned> m_OnPathPlanned;

public:
	PathCoordinator(const Localizer& localizer, PathPlanner& planner, PathFollower& follower);
	void Update();
	void MoveTo(const Vector2& pos, float radius);
	void Reset();

	// !!! Invoked in Main Thread!
	// Here we must copy data and return.
	template<typename F>
	void BindOnPathPlanned(F f) { m_OnPathPlanned.push_back(f); }

private:
	bool CheckPathObstruction() const;
	bool CheckCurrentPoseObstruction() const;

	bool PlanAndAssignPath();
	bool PlanAndAssignUnobstructPath();

	void SwitchToState(State newState);

	// Checks obstruction of current pose, then assigns unobstruction path or normal path, and switches state.
	// This function is called from many states.
	void ResolvePathAndState();
};