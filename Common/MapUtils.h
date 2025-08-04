#pragma once

#include <string>

#include "Map.h"
#include "Vector2.h"

namespace MapUtils
{
	void FillCircle(Map& map, float cellSize, Vector2 centerWorld, float radiusWorld, Map::TCellData value);
	void FillRect(Map& map, float cellSize, Vector2 minsWorld, Vector2 maxsWorld, Map::TCellData value);

	char CellDataToChar(Map::TCellData value);
	void MapToString(const Map* pMap, std::string& outString);
}