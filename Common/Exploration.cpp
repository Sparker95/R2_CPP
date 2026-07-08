#include "Exploration.h"

#include "PathFinding.h"

#include "TracyPlug.h"

#include <deque>
#include <unordered_set>
#include <array>
#include <algorithm>

using TCellPos = Exploration::TCellPos;
using uint = unsigned int;

ExpMap Exploration::CalculateExplorationMap(const OccupancyMap& map, const DistMap& distMap,
	const Vector2& startPos_w, float distMapThreshold_w,
	std::unordered_set<TCellPos>& outExpCells)
{
	ZoneScopedN("Exploration::CalculateExplorationMap");

	TCellPos startCell = map.PosWorldToMap(startPos_w);

	float distMapThreshold_m = PathFinding::WorldToDistanceMapValue(distMapThreshold_w, map.GetResolution_w());

	ExpMap expMap(map.GetSize().x, map.GetSize().y, map.GetCenter().x, map.GetCenter().y, map.GetResolution_w());
	const ExpMap::TCellData EXP_MAP_VALUE_UNVISITED = static_cast<ExpMap::TCellData>(-1);
	expMap.SetAllData(EXP_MAP_VALUE_UNVISITED);

	if (!expMap.IsMapPosWithinBounds(startCell))
		return expMap;

	expMap.SetDataAtPos(startCell, 0);
	std::deque<TCellPos> frontCells;
	frontCells.push_back(startCell);

	int neighborOffsetAndCost[8][3] = { {0,  1, 3},
										{1,  1, 4},
										{1,  0, 3},
										{1, -1, 4},
										{0, -1, 3},
										{-1,-1, 4},
										{-1, 0, 3},
										{-1, 1, 4} };

	while (!frontCells.empty())
	{
		TCellPos thisCell = frontCells.front();
		frontCells.pop_front();
		ExpMap::TCellData thisValue = expMap.GetDataAtPos(thisCell);

		for (uint i = 0; i < 8; i++)
		{
			int* offAndCost = neighborOffsetAndCost[i];

			TCellPos neighborCell(thisCell.x + offAndCost[0], thisCell.y + offAndCost[1]);

			// Ignore if out of bounds
			if (!map.IsMapPosWithinBounds(neighborCell))
				continue;

			// Ignore if occupied
			if (map.GetDataAtPos(neighborCell) > 0)
				continue;

			// Ignore if can't fit there
			if (static_cast<float>(distMap.GetDataAtPos(neighborCell)) < distMapThreshold_m)
				continue;

			ExpMap::TCellData newNeighborValue = thisValue + static_cast<ExpMap::TCellData>(offAndCost[2]);
			ExpMap::TCellData neighborValue = expMap.GetDataAtPos(neighborCell);

			if (neighborValue == EXP_MAP_VALUE_UNVISITED)
			{
				// Never visited this cell
				expMap.SetDataAtPos(neighborCell, newNeighborValue);

				// Add it to front, or to cells for exploration?
				if (map.GetDataAtPos(neighborCell) == 0)
				{
					// Cell occupancy is unknown - add it to cells for exploration
					outExpCells.insert(neighborCell);
				}
				else
				{
					// Regular most likely free cell - add it to front
					frontCells.push_back(neighborCell);
				}
			}
			else if (newNeighborValue < neighborValue)
			{
				// Already visited but more expensive
				expMap.SetDataAtPos(neighborCell, newNeighborValue);
			}
		}
	}

	return expMap;
}

uint GetCellNeighbors(TCellPos cell, const std::unordered_set<TCellPos> cellsSet,
	std::array<TCellPos, 8>& outCells)
{
	uint iOutCell = 0;
	int neighborOffsets[8][2] = { {0,  1},
								{1,  1,},
								{1,  0,},
								{1, -1,},
								{0, -1,},
								{-1,-1,},
								{-1, 0,},
								{-1, 1,} };

	for (uint i = 0; i < 8; i++)
	{
		TCellPos neighborCell = cell + TCellPos(neighborOffsets[i][0], neighborOffsets[i][1]);
		if (cellsSet.find(neighborCell) != cellsSet.end())
		{
			outCells[iOutCell] = neighborCell;
			iOutCell++;
		}
	}

	return iOutCell;
}

// Finds cells in cells which are connected to the cell
void FindConnectedCells(TCellPos cell, const std::unordered_set<TCellPos>& cells, std::unordered_set<TCellPos>& outConnectedCells)
{
	outConnectedCells.insert(cell);

	std::deque<TCellPos> que;
	que.push_back(cell);

	while (!que.empty())
	{
		TCellPos thisCell = que.front();
		que.pop_front();

		std::array<TCellPos, 8> neighborCells;
		uint nNeighbors = GetCellNeighbors(thisCell, cells, neighborCells);
		for (uint i = 0; i < nNeighbors; i++)
		{
			TCellPos neighborCell = neighborCells[i];
			if (outConnectedCells.find(neighborCell) == outConnectedCells.end())
			{
				que.push_back(neighborCell);
				outConnectedCells.insert(neighborCell);
			}
		}
	}
}

std::vector<std::unordered_set<TCellPos>> Exploration::FindCellGraphComponents(std::unordered_set<TCellPos> cells)
{
	ZoneScopedN("Exploration::FindCellGraphComponents");

	std::unordered_set<TCellPos> cellsCopy = cells; // Working copy of the set, we will modify it
	std::vector<std::unordered_set<TCellPos>> components;

	while (!cellsCopy.empty())
	{
		TCellPos anyCell = *cellsCopy.begin();
		std::unordered_set<TCellPos>& anyCellComponent = *components.emplace(components.end());
		FindConnectedCells(anyCell, cellsCopy, anyCellComponent); // Means : component of the anyCell

		// Removing elements in cellsCopy found in anyCellComponent
		for (const TCellPos& cellInComponent : anyCellComponent)
		{
			auto it = cellsCopy.find(cellInComponent);
			if (it != cellsCopy.end())
				cellsCopy.erase(it);
		}
	}

	return components;
}

bool Exploration::EvaluateCellComponent(const std::unordered_set<TCellPos>& cells,
	const ExpMap& explorationMap, float excludeDist_w, Exploration::CellComponentEval& outResult)
{
	// Closer is better, bigger number of connected cells is better
	// distScore = 1 / (1 + min dist to any cell in cellsSet)
	// compSizeScore = cellsSet.size()
	// score = compSizeScore * distScore

	TCellPos nearestCell;
	ExpMap::TCellData minDist_m = 0xFFFF; // _m stands for "Map" units!
	ExpMap::TCellData excludeDist_m = static_cast<ExpMap::TCellData>(excludeDist_w / explorationMap.GetResolution_w());

	for (const TCellPos& cell : cells)
	{
		ExpMap::TCellData dist_m = explorationMap.GetDataAtPos(cell);

		// Ignore if cell is too close
		if (dist_m < excludeDist_m)
			continue;

		if (dist_m < minDist_m)
		{
			minDist_m = dist_m;
			nearestCell = cell;
		}
	}

	// All cells are too close, this component should not be used for exploration as it's too close
	if (minDist_m == 0xFFFF)
		return false;

	float distScore = 1 / (1 + static_cast<float>(minDist_m));
	float compSizeScore = static_cast<float>(cells.size());
	float score = 1000.0f * distScore * compSizeScore; // Scale for better range

	outResult = { &cells, score, nearestCell };

	return true;
}

void Exploration::EvaluateSortCellComponents(const std::vector<std::unordered_set<TCellPos>>& cellComponents, const ExpMap& explorationMap,
	float excludeDist_w, std::vector<Exploration::CellComponentEval>& outEvalResults)
{
	outEvalResults.reserve(cellComponents.size());
	
	for (const auto& component : cellComponents)
	{
		CellComponentEval evalResult;
		if (!EvaluateCellComponent(component, explorationMap, excludeDist_w, evalResult))
			continue;
		outEvalResults.push_back(evalResult);
	}

	std::sort(outEvalResults.begin(), outEvalResults.end(), [](const CellComponentEval& a, const CellComponentEval& b)
		{
			return a.m_Score > b.m_Score;
		});
}