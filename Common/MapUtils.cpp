#include "MapUtils.h"

#include "StringWordWriter.h"

#include <fstream>

namespace MapUtils
{
	void FillCircle(Map& map, Vector2 centerWorld, float radiusWorld, Map::TCellData value)
	{
		float cellSize = map.GetResolution_w();

		Vector2 centerLocal = centerWorld / cellSize;
		float radiusLocal = radiusWorld / cellSize;

		Map::TCellPos minMap = map.PosWorldToMap(Vector2(centerLocal.x - radiusLocal, centerLocal.y - radiusLocal));
		Map::TCellPos maxMap = map.PosWorldToMap(Vector2(centerLocal.x + radiusLocal, centerLocal.y + radiusLocal));

		for (int y = minMap.y; y <= maxMap.y; y++)
		{
			for (int x = minMap.x; x <= maxMap.x; x++)
			{
				Map::TCellPos cellPosMap(x, y);
				if (!map.IsMapPosWithinBounds(cellPosMap))
					continue;

				Vector2 cellPosLocal = map.PosMapToWorld(cellPosMap);
				Vector2 diffLocal = cellPosLocal - centerLocal;

				if ((diffLocal.x * diffLocal.x + diffLocal.y * diffLocal.y) > radiusLocal*radiusLocal)
					continue;

				map.SetDataAtPos(cellPosMap, value);
			}
		}
	}

	void FillRect(Map& map, Vector2 minsWorld, Vector2 maxsWorld, Map::TCellData value)
	{
		Map::TCellPos minMap = map.PosWorldToMap(minsWorld);
		Map::TCellPos maxMap = map.PosWorldToMap(maxsWorld);
		for (int y = minMap.y; y <= maxMap.y; y++)
		{
			for (int x = minMap.x; x <= maxMap.x; x++)
			{
				Map::TCellPos cellPosMap(x, y);
				if (!map.IsMapPosWithinBounds(cellPosMap))
					continue;

				map.SetDataAtPos(cellPosMap, value);
			}
		}
	}

	bool WriteMapToFile_L2Int(const Map& map, const char* pFilePath, uint l2IntQuantization)
	{
		// Remember binary flag! Otherwise size measurement goes wrong, because of CRLF conversions.
		std::ofstream f(pFilePath, std::ios_base::out | std::ios_base::binary);
		if (!f.is_open())
			return false;

		std::string outString;
		StringWordWriter writer(outString);

		Map::TCellSize mapSize = map.GetSize();
		Map::TCellSize mapCenter = map.GetCenter();

		writer.Write("MAP");
		writer.Write("FORMAT");
		writer.Write("L2INT");
		writer.Write("QUANT");
		writer.Write<uint>(l2IntQuantization);
		writer.WriteNewLine();

		writer.Write("RES_W");
		writer.Write(map.GetResolution_w());
		
		writer.Write("XSIZE");
		writer.Write(mapSize.x);
		writer.Write("YSIZE");
		writer.Write(mapSize.y);

		writer.Write("XCENTER");
		writer.Write(mapCenter.x);
		writer.Write("YCENTER");
		writer.Write(mapCenter.y);
		writer.WriteNewLine();

		writer.Write("MAP_DATA");
		writer.WriteNewLine();

		for (uint y = 0; y < mapSize.y; y++)
		{
			writer.Write("ROW_ID");
			writer.Write(y);
			writer.Write("ROW_DATA");

			for (uint x = 0; x < mapSize.x; x++)
			{
				Map::TCellPos cellPos(x, y);
				Map::TCellData value = map.GetDataAtPos(cellPos);
				writer.Write<int>(static_cast<int>(value));
			}
			writer.WriteNewLine();
		}

		f.seekp(0);
		f.write(outString.data(), outString.size());

		return true;
	}
}
