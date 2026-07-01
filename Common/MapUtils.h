#pragma once

#include <string>

#include "Map.h"
#include "Vector2.h"
#include "StringWordWriter.h"
#include "StringWordReader.h"

#include <fstream>

namespace MapUtils
{
	template<typename TMap>
	void FillCircle(TMap& map, Vector2 centerWorld, float radiusWorld, OccupancyMap::TCellData value)
	{
		float cellSize = map.GetResolution_w();

		Vector2 centerLocal = centerWorld / cellSize;
		float radiusLocal = radiusWorld / cellSize;

		OccupancyMap::TCellPos minMap = map.PosWorldToMap(Vector2(centerLocal.x - radiusLocal, centerLocal.y - radiusLocal));
		OccupancyMap::TCellPos maxMap = map.PosWorldToMap(Vector2(centerLocal.x + radiusLocal, centerLocal.y + radiusLocal));

		for (int y = minMap.y; y <= maxMap.y; y++)
		{
			for (int x = minMap.x; x <= maxMap.x; x++)
			{
				OccupancyMap::TCellPos cellPosMap(x, y);
				if (!map.IsMapPosWithinBounds(cellPosMap))
					continue;

				Vector2 cellPosLocal = map.PosMapToWorld(cellPosMap);
				Vector2 diffLocal = cellPosLocal - centerLocal;

				if ((diffLocal.x * diffLocal.x + diffLocal.y * diffLocal.y) > radiusLocal * radiusLocal)
					continue;

				map.SetDataAtPos(cellPosMap, value);
			}
		}
	}

	template<typename TMap>
	void FillRect(TMap& map, Vector2 minsWorld, Vector2 maxsWorld, OccupancyMap::TCellData value)
	{
		OccupancyMap::TCellPos minMap = map.PosWorldToMap(minsWorld);
		OccupancyMap::TCellPos maxMap = map.PosWorldToMap(maxsWorld);
		for (int y = minMap.y; y <= maxMap.y; y++)
		{
			for (int x = minMap.x; x <= maxMap.x; x++)
			{
				OccupancyMap::TCellPos cellPosMap(x, y);
				if (!map.IsMapPosWithinBounds(cellPosMap))
					continue;

				map.SetDataAtPos(cellPosMap, value);
			}
		}
	}

	template<typename TMap>
	bool WriteMapToFile_L2Int(const TMap& map, const char* pFilePath, uint l2IntQuantization)
	{
		// Remember binary flag! Otherwise size measurement goes wrong, because of CRLF conversions.
		std::ofstream f(pFilePath, std::ios_base::out | std::ios_base::binary);
		if (!f.is_open())
			return false;

		std::string outString;
		StringWordWriter writer(outString);

		OccupancyMap::TCellSize mapSize = map.GetSize();
		OccupancyMap::TCellSize mapCenter = map.GetCenter();

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
				OccupancyMap::TCellPos cellPos(x, y);
				OccupancyMap::TCellData value = map.GetDataAtPos(cellPos);
				writer.Write<int>(static_cast<int>(value));
			}
			writer.WriteNewLine();
		}

		f.seekp(0);
		f.write(outString.data(), outString.size());

		return true;
	}

	template<typename TMap>
	bool ReadMapFromFile_L2Int(const char* pFilePath, TMap& outMap, uint& outl2IntQuantization)
	{
		// Remember binary flag! Otherwise size measurement goes wrong, because of CRLF conversions.
		std::ifstream f(pFilePath, std::ios_base::in | std::ios_base::binary);
		if (!f.is_open())
			return false;

		// Read whole file into memory
		f.seekg(0, std::ios::end);
		uint fileSize = static_cast<uint>(f.tellg());
		f.seekg(0, std::ios::beg);

		std::string fileContents;
		fileContents.resize(fileSize, 0);
		f.read(fileContents.data(), fileSize);

		StringWordReader reader(fileContents.data(), fileSize);

		// Read data

		if (!reader.ReadCompareWord("MAP"))
			return false;

		if (!reader.ReadCompareWord("FORMAT"))
			return false;

		if (!reader.ReadCompareWord("L2INT"))
			return false;

		if (!reader.ReadCompareWord("QUANT"))
			return false;

		if (!reader.Read<uint>(outl2IntQuantization))
			return false;

		if (!reader.ReadCompareWord("RES_W"))
			return false;

		float mapRes_w;
		if (!reader.Read<float>(mapRes_w))
			return false;

		uint xSize, ySize;
		if (!reader.ReadCompareWord("XSIZE"))
			return false;
		if (!reader.Read<uint>(xSize))
			return false;

		if (!reader.ReadCompareWord("YSIZE"))
			return false;
		if (!reader.Read<uint>(ySize))
			return false;

		uint xCenter, yCenter;
		if (!reader.ReadCompareWord("XCENTER"))
			return false;
		if (!reader.Read<uint>(xCenter))
			return false;

		if (!reader.ReadCompareWord("YCENTER"))
			return false;
		if (!reader.Read<uint>(yCenter))
			return false;

		if (!reader.ReadCompareWord("MAP_DATA"))
			return false;

		outMap = OccupancyMap(xSize, ySize, xCenter, yCenter, mapRes_w);

		for (uint y = 0; y < ySize; y++)
		{
			if (!reader.ReadCompareWord("ROW_ID"))
				return false;

			uint _y;
			if (!reader.Read<uint>(_y))
				return false;
			if (_y != y)
				return false;

			if (!reader.ReadCompareWord("ROW_DATA"))
				return false;

			for (uint x = 0; x < xSize; x++)
			{
				int cellData;
				if (!reader.Read<int>(cellData))
					return false;

				if ((cellData < -128) || (cellData > 127))
					return false;

				outMap.SetDataAtPos(OccupancyMap::TCellPos(x, y), static_cast<OccupancyMap::TCellData>(cellData));
			}
		}

		return true;
	}
}
