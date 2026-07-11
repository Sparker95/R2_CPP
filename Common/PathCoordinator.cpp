#include "PathCoordinator.h"
#include "PathPlanner.h"
#include "Localizer.h"

#include "TracyPlug.h"

#include <cstdio>

const char* PathCoordinator::s_aStateText[5] = {
	"IDLE",
	"MOVING",
	"MOVING_UNOBSTRUCT",
	"PATH_OBSTR_STOPPING",
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
	m_pPathFollower->ResetAndStop();
}

void PathCoordinator::ResolvePathAndState()
{
	if (!CheckCurrentPoseObstruction())
	{
		printf("PathCoordinator: Current pose is obstructed.\n");
		if (PlanAndAssignUnobstructPath())
			SwitchToState(State::MOVING_UNOBSTRUCT);
		else
			SwitchToState(State::FAIL);
	}
	else if (PlanAndAssignPath())
		SwitchToState(State::MOVING);
	else
		SwitchToState(State::FAIL);
}

void PathCoordinator::Update()
{
	ZoneScopedN("PathCoordinator::Update");

	switch (m_State)
	{
		case State::IDLE:
		{
			if (m_MoveToPosRadius == 0)
				return;

			// New movement has been requested. First ensure we stopped.
			if (m_pPathFollower->GetState() == PathFollower::State::MOVING ||
				m_pPathFollower->GetState() == PathFollower::State::ERROR)
			{
				m_pPathFollower->ResetAndStop();
				printf("PathCoordinator: New movement requested, requesting ResetAndStop.\n");
				return;
			}
			else if (m_pPathFollower->GetState() == PathFollower::State::STOPPING)
			{
				printf("PathCoordinator: New movement requested, waiting for stop.\n");
				return;
			}

			ResolvePathAndState();

			break;
		}
		case State::MOVING:
		{
			PathFollower::State followerState = m_pPathFollower->GetState();

			if (followerState == PathFollower::State::ERROR)
			{
				printf("PathCoordinator: PathFollower error.\n");
				SwitchToState(State::FAIL);
			}
			else if (followerState == PathFollower::State::IDLE)
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
					m_pPathFollower->ResetAndStop();
					SwitchToState(State::PATH_OBSTR);
				}
			}
			break;
		}
		case State::MOVING_UNOBSTRUCT:
		{
			PathFollower::State followerState = m_pPathFollower->GetState();

			if (followerState == PathFollower::State::ERROR)
			{
				printf("PathCoordinator: PathFollower error.\n");
				SwitchToState(State::FAIL);
			}
			else if (followerState == PathFollower::State::IDLE)
			{
				printf("PathCoordinator: PathFollower has finished unobstruction path.\n");
				
				ResolvePathAndState();
			}
			break;
		}
		case State::PATH_OBSTR:
		{
			if (m_pPathFollower->GetState() != PathFollower::State::IDLE)
			{
				printf("PathCoordinator: Path is obstructed, waiting for stop.\n");
				return;
			}

			printf("PathCoordinator: Braking finished.\n");

			printf("PathCoordinator: Out of the damn way!\n");

			ResolvePathAndState();
			break;
		}
		case State::FAIL:
		{
			break;
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

	const OccupancyMap& map = m_pLocalizer->GetRealTimeMap();
	const Pose& pose = m_pLocalizer->GetMeanPose();

	m_pPathPlanner->PlanPath(map, pose, m_MoveToPos, m_MoveToPosRadius);

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
		m_pPathFollower->ResetAndStop();

		// Invoke OnPathPlanned
		std::vector<PathFollower::PathPoint> emptyPath;
		for (const auto& f : m_OnPathPlanned)
			f(emptyPath);

		return false;
	}
}

bool PathCoordinator::PlanAndAssignUnobstructPath()
{
	printf("PathCoordinator::PlanAndAssignUnobstructPath()\n");

	const OccupancyMap& map = m_pLocalizer->GetRealTimeMap();
	const Pose& pose = m_pLocalizer->GetMeanPose();

	m_pPathPlanner->PlanUnobstructPath(map, pose);

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
		m_pPathFollower->ResetAndStop();

		// Invoke OnPathPlanned
		std::vector<PathFollower::PathPoint> emptyPath;
		for (const auto& f : m_OnPathPlanned)
			f(emptyPath);

		return false;
	}
}

bool PathCoordinator::CheckPathObstruction() const
{
	const OccupancyMap& map = m_pLocalizer->GetRealTimeMap();

	const auto& path = m_pPathFollower->GetPath();
	uint currentPathPointId = m_pPathFollower->GetCurrentPointId();
	uint obstrPathPointId = -1;
	
	bool result = m_pPathPlanner->CheckPathObstruction(map, path, currentPathPointId, &obstrPathPointId);

	if (!result)
		printf("PathCoordinator: Path is obstructed, point %u / %u.\n", obstrPathPointId, (uint)path.size());

	return result;
}

bool PathCoordinator::CheckCurrentPoseObstruction() const
{
	const OccupancyMap& map = m_pLocalizer->GetRealTimeMap();
	const Pose& pose = m_pLocalizer->GetMeanPose();

	return m_pPathPlanner->CheckPoseObstruction(map, pose);
}