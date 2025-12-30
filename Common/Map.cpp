#include "Map.h"

// Tracy
#include "Tracy/tracy/Tracy.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <cstring>

#include "Vector2.h"

Map::Map(float resolution_w) :
	m_Res_w(resolution_w),
	m_xSize(SIZE_INITIAL),
	m_ySize(SIZE_INITIAL)
{
	ZoneScopedN("Map::Map");

	m_Data = new TCellData[m_xSize * m_ySize];

	m_xCenter = m_xSize / 2;
	m_yCenter = m_ySize / 2;

	// Init data
	uint ttlSize = m_xSize * m_ySize;
	std::memset(m_Data, CELL_VALUE_DEFAULT, ttlSize);
}

// Copy
Map::Map(const Map& other) noexcept
{
	ZoneScopedN("Map::Copy");

	m_Res_w = other.m_Res_w;
	m_xSize = other.m_xSize;
	m_ySize = other.m_ySize;
	m_xCenter = other.m_xCenter;
	m_yCenter = other.m_yCenter;

	m_Data = new TCellData[m_xSize * m_ySize];

	// memcpy takes majority of time here: ~100us
	std::memcpy(m_Data, other.m_Data, m_xSize * m_ySize);
}

// Assignment
Map& Map::operator=(const Map& other) noexcept
{
	ZoneScopedN("Map::Assign");

	if (m_Data)
	{
		if (m_xSize * m_ySize != other.m_xSize * other.m_ySize)
		{
			delete[] m_Data;
			m_Data = nullptr;
		}
	}

	m_Res_w = other.m_Res_w;
	m_xSize = other.m_xSize;
	m_ySize = other.m_ySize;
	m_xCenter = other.m_xCenter;
	m_yCenter = other.m_yCenter;

	if (!m_Data)
		m_Data = new TCellData[m_xSize * m_ySize];

	std::memcpy(m_Data, other.m_Data, m_xSize * m_ySize);
	
	return *this;
}

// Move
Map::Map(Map&& other) noexcept
{
	ZoneScopedN("Map::Move");

	m_Res_w = other.m_Res_w;
	m_xSize = other.m_xSize;
	m_ySize = other.m_ySize;
	m_xCenter = other.m_xCenter;
	m_yCenter = other.m_yCenter;

	m_Data = other.m_Data;
	other.m_Data = nullptr;
}

Map::~Map()
{
	ZoneScopedN("Map::~Map");
	if (m_Data)
		delete[] m_Data;
}

Map::TCellData Map::GetDataAtPos(TCellPos mapPos) const
{
#ifdef _DEBUG
	if (!IsMapPosWithinBounds(mapPos))
	{
		std::fprintf(stderr, "Map: Error: GetDataAtPos: Invalid map position: %i, %i\n", mapPos.x, mapPos.y);
		TCellData _default = {};
		throw std::runtime_error("Map: GetDataAtPos: Index is out of map bounds\n");
		return _default;
	}
#endif

	return m_Data[m_xSize * mapPos.y + mapPos.x];
}

void Map::SetDataAtPos(TCellPos mapPos, TCellData value)
{
#ifdef _DEBUG
	if (!IsMapPosWithinBounds(mapPos))
	{
		std::fprintf(stderr, "Map: Error: GetDataAtPos: Invalid map position: %i, %i\n", mapPos.x, mapPos.y);
		throw std::runtime_error("Map: SetDataAtPos: Index is out of map bounds\n");
		return;
	}
#endif

	m_Data[m_xSize * mapPos.y + mapPos.x] = value;
}

void Map::SetAllData(TCellData value)
{
	if (!m_Data)
		return;

	uint ttlSize = m_xSize * m_ySize;
	std::memset(m_Data, value, ttlSize);
}

Vector2 Map::PosMapToWorld(TCellPos mapPos) const
{
	float halfres = 0.5f * m_Res_w;
	int dxMap = mapPos.x - m_xCenter;
	int dyMap = mapPos.y - m_yCenter;

	return Vector2(static_cast<float>(dxMap) * m_Res_w + halfres,
		static_cast<float>(dyMap) * m_Res_w + halfres);
}

Vector2 Map::PosMapToWorldUnscaled(TCellPos mapPos) const
{
	float halfCell = 0.5f;
	int dxMap = mapPos.x - m_xCenter;
	int dyMap = mapPos.y - m_yCenter;

	return Vector2(static_cast<float>(dxMap) + halfCell,
		static_cast<float>(dyMap) + halfCell);
}

Map::TCellPos Map::PosWorldToMap(Vector2 pos) const
{
	return TCellPos(m_xCenter + static_cast<int>(floorf(pos.x / m_Res_w)),
		m_yCenter + static_cast<int>(floorf(pos.y / m_Res_w)));
}

Map::TCellPos Map::PosWorldUnscaledToMap(Vector2 pos_u) const
{
	return TCellPos(m_xCenter + static_cast<int>(floorf(pos_u.x)),
		m_yCenter + static_cast<int>(floorf(pos_u.y)));
}

void Map::GetWorldBounds(Vector2& outMins, Vector2& outMaxs) const
{
	outMins.x = -m_Res_w * static_cast<float>(m_xCenter);
	outMins.y = -m_Res_w * static_cast<float>(m_yCenter);

	outMaxs.x = m_Res_w * static_cast<float>(m_xSize - m_xCenter);
	outMaxs.y = m_Res_w * static_cast<float>(m_ySize - m_yCenter);
}

bool Map::IsMapPosWithinBounds(TCellPos mapPos) const
{
	return mapPos.x >= 0 && mapPos.x < static_cast<int>(m_xSize) &&
		mapPos.y >= 0 && mapPos.y < static_cast<int>(m_ySize);
}

void Map::EnsureWorldBounds_Rect(Vector2 mins_w, Vector2 maxs_w)
{
	ZoneScopedN("Map::EnsureWorldBounds");

	TCellPos mins = PosWorldToMap(mins_w);
	TCellPos maxs = PosWorldToMap(maxs_w);

	mins.x = std::min(mins.x, 0);
	maxs.x = std::max(maxs.x, static_cast<int>(m_xSize));
	mins.y = std::min(mins.y, 0);
	maxs.y = std::max(maxs.y, static_cast<int>(m_ySize));

	bool extendRight = maxs.x > static_cast<int>(m_xSize);
	bool extendLeft = mins.x < 0;
	bool extendUp = maxs.y > static_cast<int>(m_ySize);
	bool extendDown = mins.y < 0;

	if (!extendRight && !extendLeft && !extendUp && !extendDown)
		return; // No need to increase size

	// Calculate new size, find new center position.
	uint newxSize = m_xSize;
	uint newySize = m_ySize;
	uint newxCenter = m_xCenter;
	uint newyCenter = m_yCenter;

	if (extendLeft || extendRight)
	{
		newxSize = SIZE_INCREMENT + m_xSize; // Ensure size is always increased
		while (newxSize < static_cast<uint>(maxs.x - mins.x + 1))
			newxSize = SIZE_INCREMENT + newxSize;

		newxCenter = m_xCenter;

		if (extendLeft && extendRight)
		{
			uint newxCenterInNewBounds = m_xCenter + static_cast<uint>(-mins.x); // xmin is negative
			newxCenter = newxCenterInNewBounds + int((newxSize - static_cast<uint>(maxs.x - mins.x)) >> 1); // Equally extend both ways
		}
		else if (extendLeft)
			newxCenter = newxSize - (m_xSize - m_xCenter);
	}

	if (extendDown || extendUp)
	{
		newySize = SIZE_INCREMENT + m_ySize; // Ensure size is always increased
		while (newySize < static_cast<uint>(maxs.y - mins.y + 1))
			newySize = SIZE_INCREMENT + newySize;

		newyCenter = m_yCenter;

		if (extendDown && extendUp)
		{
			uint newyCenterInNewBounds = m_yCenter + static_cast<uint>(-mins.y); // ymin is negative
			newyCenter = newyCenterInNewBounds + int((newySize - static_cast<uint>(maxs.y - mins.y)) >> 1); // Equally extend both ways
		}
		else if (extendDown)
			newyCenter = newySize - (m_ySize - m_yCenter);
	}

	// Copy data
	TCellData* newData = new TCellData[newxSize * newySize];
	std::memset(newData, CELL_VALUE_DEFAULT, newxSize * newySize);

	uint xCopyStartDst = newxCenter - m_xCenter;
	//uint xCopyEnd = newxCenter + m_xSize - m_xCenter;
	uint yCopyStartDst = newyCenter - m_yCenter;
	//uint yCopyEnd = newyCenter + m_ySize - m_yCenter;
	
	for (uint ySrc = 0; ySrc < m_ySize; ySrc++)
	{
		uint yDst = yCopyStartDst + ySrc;
		std::memcpy(&newData[newxSize * yDst + xCopyStartDst], &m_Data[m_xSize * ySrc], m_xSize);
	}

	// Update member variables
	delete[] m_Data;
	m_Data = newData;
	m_xCenter = newxCenter;
	m_yCenter = newyCenter;
	m_xSize = newxSize;
	m_ySize = newySize;
}

void Map::EnsureWorldBounds_Circle(Vector2 center_w, float radius_w)
{
	Vector2 mins_w(center_w.x - radius_w, center_w.y - radius_w);
	Vector2 maxs_w(center_w.x + radius_w, center_w.y + radius_w);
	EnsureWorldBounds_Rect(mins_w, maxs_w);
}