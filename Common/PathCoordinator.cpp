#include "PathCoordinator.h"
#include "PathPlanner.h"
#include "Localizer.h"

#include "TracyPlug.h"

#include <cstdio>

const char* PathCoordinator::s_aStateText[5] = {
	"IDLE",
	"MOVING",
	"PATH_OBSTR_BRAKING",
	"PATH_OBSTR_WAITING_CLEAR",
	"FAIL"
};

PathCoordinator::PathCoordinator(const Localizer& localizer, PathPlanner& planner, PathFollower& follower)
	: m_pLocalizer(&localizer),
	m_pPathPlanner(&planner),
	m_pPathFollower(&follower)
{
}

void PathCoordinator::MoveTo(const Vector2& pos, float radius)
{
	printf("PathCoordinator::MoveTo(): %.2f %.2f r%.2f\n", pos.x, pos.y, radius);
	m_MoveToPos = pos;
	m_MoveToPosRadius = radius;
	SwitchToState(State::IDLE);
}

void PathCoordinator::Reset()
{
	printf("PathCoordinator::Reset()\n");
	m_MoveToPosRadius = 0;
	m_MoveToPos = Vector2::ZERO;
	SwitchToState(State::IDLE);
	m_pPathFollower->Reset();
}

void PathCoordinator::Update(const ControlSensorFrame& frame)
{
	ZoneScopedN("PathCoordinator::Update");

	if (m_State == State::IDLE)
	{
		if (m_MoveToPosRadius == 0)
			return;

		// New movement has been requested. First ensure we stopped.
		if (frame.IsMoving())
		{
			printf("PathCoordinator: New movement requested, braking.\n");
			return;
		}

		if (PlanAndAssignPath())
		{
			SwitchToState(State::MOVING);
		}
		else
		{
			SwitchToState(State::FAIL);
		}
	}
	else if (m_State == State::MOVING)
	{
		PathFollower::State followerState = m_pPathFollower->GetState();

		if (followerState == PathFollower::State::ERROR)
		{
			printf("PathCoordinator: PathFollower error.\n");
			SwitchToState(State::FAIL);
		}
		else if (followerState == PathFollower::State::END || followerState == PathFollower::State::IDLE)
		{
			printf("PathCoordinator: PathFollower has finished.\n");
			Reset();
		}
		else
		{
			// Still moving, check path obstruction
			if (!CheckPathObstruction())
			{
				printf("PathCoordinator: Path is obstructed, braking.\n");
				m_pPathFollower->Reset();
				SwitchToState(State::PATH_OBSTR_BRAKING);
			}
		}
	}
	else if (m_State == State::PATH_OBSTR_BRAKING)
	{
		if (frame.IsMoving())
		{
			printf("PathCoordinator: Path is obstructed, waiting for stop.\n");
			return;
		}

		printf("PathCoordinator: Braking finished.\n");
		SwitchToState(State::PATH_OBSTR_WAITING_CLEAR);
	}
	else if (m_State == State::PATH_OBSTR_WAITING_CLEAR)
	{
		printf("PathCoordinator: Out of the damn way!\n");

		if (PlanAndAssignPath())
		{
			printf("PathCoordinator: Found alternative path.\n");
			SwitchToState(State::MOVING);
		}
		else
		{
			printf("PathCoordinator: Failed to find alternative path.\n");
			SwitchToState(State::FAIL);
		}
	}
}

void PathCoordinator::SwitchToState(State newState)
{
	m_State = newState;

	printf("PathCoordinator::SwitchToState: %s\n", s_aStateText[(uint)newState]);
}

bool PathCoordinator::PlanAndAssignPath()
{
	printf("PathCoordinator::PlanAndAssignPath()\n");

	const Localizer::Particle& locParticle = m_pLocalizer->GetMostLikelyParticle();

	m_pPathPlanner->PlanPath(locParticle.m_Map, locParticle.m_Pose, m_MoveToPos, m_MoveToPosRadius);

	if (m_pPathPlanner->PathValid())
	{
		const auto& path = m_pPathPlanner->GetPath();

		m_pPathFollower->FollowPath(path);

		// Invoke OnPathPlanned
		for (const auto& f : m_OnPathPlanned)
			f(path);

		return true;
	}
	else
	{
		m_pPathFollower->Reset();

		// Invoke OnPathPlanned
		std::vector<PathFollower::PathPoint> emptyPath;
		for (const auto& f : m_OnPathPlanned)
			f(emptyPath);

		return false;
	}
}

bool PathCoordinator::CheckPathObstruction()
{
	const Localizer::Particle& locParticle = m_pLocalizer->GetMostLikelyParticle();

	const auto& path = m_pPathFollower->GetPath();
	uint currentPathPointId = m_pPathFollower->GetCurrentPointId();
	uint obstrPathPointId = -1;
	
	bool result = m_pPathPlanner->CheckPathObstruction(locParticle.m_Map, path, currentPathPointId, &obstrPathPointId);

	if (!result)
		printf("PathCoordinator: Path is obstructed, point %u / %u.\n", obstrPathPointId, (uint)path.size());

	return result;
}

