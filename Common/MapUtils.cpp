#include "MapUtils.h"

namespace MapUtils
{
	void FillCircle(Map& map, float cellSize, Vector2 centerWorld, float radiusWorld, Map::TCellData value)
	{
		Vector2 centerLocal = centerWorld / cellSize;
		float radiusLocal = radiusWorld / cellSize;

		Map::CellPos minMap = Map::GetCellIndex(Vector2(centerLocal.x - radiusLocal, centerLocal.y - radiusLocal));
		Map::CellPos maxMap = Map::GetCellIndex(Vector2(centerLocal.x + radiusLocal, centerLocal.y + radiusLocal));

		for (int y = minMap.y; y <= maxMap.y; y++)
		{
			for (int x = minMap.x; x <= maxMap.x; x++)
			{
				Map::CellPos cellPosMap(x, y);
				if (!map.IsCellPosWithinBounds(cellPosMap))
					continue;

				Vector2 cellPosLocal = Map::GetCellCenterPos(cellPosMap);
				Vector2 diffLocal = cellPosLocal - centerLocal;

				if ((diffLocal.x * diffLocal.x + diffLocal.y * diffLocal.y) > radiusLocal*radiusLocal)
					continue;

				map.SetCellData(cellPosMap, value);
			}
		}
	}

	void FillRect(Map& map, float cellSize, Vector2 minsWorld, Vector2 maxsWorld, Map::TCellData value)
	{
		Vector2 minsLocal = minsWorld / cellSize;
		Vector2 maxsLocal = maxsWorld / cellSize;

		Map::CellPos minMap = Map::GetCellIndex(minsLocal);
		Map::CellPos maxMap = Map::GetCellIndex(maxsLocal);
		for (int y = minMap.y; y <= maxMap.y; y++)
		{
			for (int x = minMap.x; x <= maxMap.x; x++)
			{
				Map::CellPos cellPosMap(x, y);
				if (!map.IsCellPosWithinBounds(cellPosMap))
					continue;

				map.SetCellData(cellPosMap, value);
			}
		}
	}

	char CellDataToChar(Map::TCellData value)
	{
		if (value < Map::CELL_VALUE_UNKNOWN)
			return 'X';
		else if (value > Map::CELL_VALUE_UNKNOWN)
			return '-';
		else return '?';

		/*
		const char HexChars[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
			'A', 'B', 'C', 'D', 'E', 'F' };

		int lowestHalfByte = 0xF & static_cast<int>(value >> 4);
		return HexChars[lowestHalfByte];
		*/
	}

	void MapToString(const Map* pMap, std::string& outString)
	{
		int sizeMap = pMap->GetSize();
		int nCharsExpected = sizeMap * (sizeMap + 1); // +1 because of newlines
		outString.reserve(nCharsExpected);

		for (int y = sizeMap-1; y >= 0; y--) // Iterate Y in reverse, because bigger Y are printed at top
		{
			for (int x = 0; x < sizeMap; x++)
			{
				Map::TCellData cellData = pMap->GetCellData(Map::CellPos(x, y));
				char c = CellDataToChar(cellData);
				outString.push_back(c);
			}
			outString.push_back('\n');
		}
	}
}
