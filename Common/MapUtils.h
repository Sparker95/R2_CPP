#pragma once

#include <string>

#include "Map.h"
#include "Vector2.h"

namespace MapUtils
{
	void FillCircle(Map& map, Vector2 centerWorld, float radiusWorld, Map::TCellData value);
	void FillRect(Map& map, Vector2 minsWorld, Vector2 maxsWorld, Map::TCellData value);

	bool WriteMapToFile_L2Int(const Map& map, const char* pFilePath, uint l2IntQuantization);
    bool ReadMapFromFile_L2Int(const char* pFilePath, Map& outMap, uint& outl2IntQuantization);
}
