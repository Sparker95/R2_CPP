#include "Explorer.h"

#include "Pose.h"
#include "PathCoordinator.h"
#include "PathFinding.h"
#include "Exploration.h"
#include "Localizer.h"

#include <unordered_map>
#include <cstdio>

using uint = unsigned int;

Explorer::Explorer(Localizer& localizer, PathCoordinator& pathCoordinator, const ExplorerConfig& config)
	: m_pLocalizer(&localizer),
	m_pPathCoordinator(&pathCoordinator),
	m_Config(config)
{
}

void Explorer::StartExploration()
{
	if (m_State == State::ACTIVE)
		return;

	if (FindExplorePosAndMove())
		m_State = State::ACTIVE;
	else
		m_State = State::ERROR;
}

bool Explorer::FindExplorePosAndMove()
{
	printf("Explorer:: FindExplorePosAndMove()\n");

	const OccupancyMap& map = m_pLocalizer->GetRealTimeMap();
	const Pose& pose = m_pLocalizer->GetMeanPose();

	DistMap distMap = PathFinding::CalculateDistanceMap(map, 1); // Threshold = 1->unknown space is not obstacle

	std::unordered_set<Exploration::TCellPos> expCells;
	ExpMap expMap = Exploration::CalculateExplorationMap(map, distMap, Vector2(pose.x, pose.y),
		m_Config.m_DistMapThreshold, expCells);

	if (expCells.empty())
	{
		printf("Explorer: Found no cells for exploration\n");
		return false;
	}

	std::vector<std::unordered_set<Exploration::TCellPos>> cellComponents = Exploration::FindCellGraphComponents(expCells);

	std::vector<Exploration::CellComponentEval> componentsEval;
	Exploration::EvaluateSortCellComponents(cellComponents, expMap, m_Config.m_MinDistToCell, componentsEval);

	const Exploration::CellComponentEval& bestComp = componentsEval[0];
	Vector2 bestCompPos = map.PosMapToWorld(bestComp.m_NearestCell);
	printf("Explorer: Found %u cell components for exploration\n", (uint)cellComponents.size());
	printf("Explorer: Selected component at: %.1f %.1f, cells: %u, score: %.1f\n", bestCompPos.x, bestCompPos.y,
		(uint)bestComp.m_pComponent->size(), bestComp.m_Score);

	m_pPathCoordinator->MoveTo(bestCompPos, m_Config.m_MoveRadius);

	return true;
}

void Explorer::Update()
{
	switch (m_State)
	{
		case State::IDLE:
		case State::ERROR:
			break;
		case State::ACTIVE:
		{
			using TCoordState = PathCoordinator::State;
			TCoordState coordinatorState = m_pPathCoordinator->GetState();

			switch (coordinatorState)
			{
				case TCoordState::COMPLETED:
				{
					printf("Explorer:: Arrived to exploration position. Finding another one.\n");
					if (FindExplorePosAndMove())
						m_State = State::ACTIVE;
					else
						m_State = State::ERROR;
					break;
				}
				case TCoordState::ERROR:
				{
					m_State = State::ERROR;
					break;
				}
				default:
					break;
			}
			break;
		}
	}
}