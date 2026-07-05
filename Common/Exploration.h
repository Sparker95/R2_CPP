// Algorithms for greedy exploration of the map
#pragma once

#include "Map.h"
#include "Vector2.h"

#include <vector>
#include <unordered_set>

namespace Exploration
{
	using TCellPos = OccupancyMap::TCellPos;

	ExpMap CalculateExplorationMap(const OccupancyMap& map, const DistMap& distMap,
		const Vector2& startPos_w, float distMapThreshold_w,
		std::unordered_set<TCellPos>& outExpCells);

	std::vector<std::unordered_set<TCellPos>> FindCellGraphComponents(std::unordered_set<TCellPos> cells);

	// Result of scoring and evaluation of a cell component
	struct CellComponentEval
	{
		const std::unordered_set<TCellPos>* m_pComponent;
		float m_Score;
		TCellPos m_NearestCell;
	};

	// Returns score and nearest cell in the cell component
	CellComponentEval EvaluateCellComponent(const std::unordered_set<TCellPos>& cells, const ExpMap& explorationMap);

	// Scores cell components and sorts them by score. Highest score component will be first in array.
	void EvaluateSortCellComponents(const std::vector<std::unordered_set<TCellPos>>& cellComponents, const ExpMap& explorationMap,
		std::vector<CellComponentEval>& outEvalResults);
}