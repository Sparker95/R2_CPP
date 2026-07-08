#include "Explorer.h"

#include "Pose.h"
#include "PathCoordinator.h"
#include "PathFinding.h"
#include "Exploration.h"

#include <unordered_map>
#include <cstdio>

using uint = unsigned int;

Explorer::Explorer(PathCoordinator& pathCoordinator, const ExplorerConfig& config)
	: m_pPathCoordinator(&pathCoordinator),
	m_Config(config)
{
}

void Explorer::ExploreNextPosition(const OccupancyMap& map, const Pose& pose)
{
	DistMap distMap = PathFinding::CalculateDistanceMap(map, 1); // Threshold = 1->unknown space is not obstacle

	std::unordered_set<Exploration::TCellPos> expCells;
	ExpMap expMap = Exploration::CalculateExplorationMap(map, distMap, Vector2(pose.x, pose.y),
		m_Config.m_DistMapThreshold, expCells);

	if (expCells.empty())
	{
		printf("Explorer: Found no cells for exploration\n");
		return;
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
}