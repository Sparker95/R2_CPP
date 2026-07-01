#pragma once

#include "CommonIntTypes.h"
#include "Vector2.h"

#include "TracyPlug.h"

#include <cmath>
#include <cstdio>
#include <iostream>
//#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <cstdint>

// T - type of cell data
template<typename T>
class GridMap // Should be named: GridMap
{
public:
	using TCellData = T;
	using TCellPos = Vector2Base<int>;   // Map's native position type. Cell index within [0,0 .. xSize-1,ySize-1]
	using TCellSize = Vector2Base<uint>; // Map's native size type. Size can not be negative.

	constexpr static TCellData CELL_VALUE_DEFAULT = 0;
	constexpr static uint SIZE_INITIAL = 128;
	constexpr static uint SIZE_INCREMENT = 128;

private:

	TCellData* m_Data = nullptr;
	float m_Res_w = 0;
	uint m_xSize = 0;
	uint m_ySize = 0;
	uint m_xCenter = 0;
	uint m_yCenter = 0;

public:

	GridMap() = default;

	// ##########################################################################################
	// CONSTRUCTORS, DESTRUCTOR, ASSIGNMENT, ETC

	GridMap(float resolution_w) :
		m_Res_w(resolution_w),
		m_xSize(SIZE_INITIAL),
		m_ySize(SIZE_INITIAL)
	{
		ZoneScopedN("GridMap::GridMap");

		m_Data = new TCellData[m_xSize * m_ySize];

		m_xCenter = m_xSize / 2;
		m_yCenter = m_ySize / 2;

		// Init data
		uint ttlSize = m_xSize * m_ySize;
		std::memset(m_Data, CELL_VALUE_DEFAULT, ttlSize);
	}

	GridMap(uint xSize, uint ySize, uint xCenter, uint yCenter, float resolution_w) :
		m_Res_w(resolution_w),
		m_xSize(xSize),
		m_ySize(ySize),
		m_xCenter(xCenter),
		m_yCenter(yCenter)
	{
		ZoneScopedN("GridMap::GridMap");

		m_Data = new TCellData[m_xSize * m_ySize];

		// Init data
		uint ttlSize = m_xSize * m_ySize;
		std::memset(m_Data, CELL_VALUE_DEFAULT, ttlSize);
	}

	// Copy
	GridMap(const GridMap& other) noexcept
	{
		ZoneScopedN("GridMap::Copy");

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
	GridMap& operator=(const GridMap& other) noexcept
	{
		ZoneScopedN("GridMap::Assign");

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
	GridMap(GridMap&& other) noexcept
	{
		ZoneScopedN("GridMap::Move");

		m_Res_w = other.m_Res_w;
		m_xSize = other.m_xSize;
		m_ySize = other.m_ySize;
		m_xCenter = other.m_xCenter;
		m_yCenter = other.m_yCenter;

		m_Data = other.m_Data;
		other.m_Data = nullptr;
	}

	// Destructor
	~GridMap()
	{
		ZoneScopedN("GridMap::~GridMap");
		if (m_Data)
			delete[] m_Data;
	}

	// ##########################################################################################
	// DATA ACCESS

	TCellData* GetData() { return m_Data; }

	const TCellData* GetData() const { return m_Data; }

	uint GetDataSizeBytes() const { return m_xSize * m_ySize * sizeof(TCellData); }

	TCellData GetDataAtPos(TCellPos mapPos) const
	{
#ifdef _DEBUG
		if (!IsMapPosWithinBounds(mapPos))
		{
			std::fprintf(stderr, "GridMap: Error: GetDataAtPos: Invalid map position: %i, %i\n", mapPos.x, mapPos.y);
			TCellData _default = {};
			throw std::runtime_error("GridMap: GetDataAtPos: Index is out of map bounds\n");
			return _default;
		}
#endif

		return m_Data[m_xSize * mapPos.y + mapPos.x];
	}

	void SetDataAtPos(TCellPos mapPos, TCellData value)
	{
#ifdef _DEBUG
		if (!IsMapPosWithinBounds(mapPos))
		{
			std::fprintf(stderr, "GridMap: Error: GetDataAtPos: Invalid map position: %i, %i\n", mapPos.x, mapPos.y);
			throw std::runtime_error("GridMap: SetDataAtPos: Index is out of map bounds\n");
			return;
		}
#endif

		m_Data[m_xSize * mapPos.y + mapPos.x] = value;
	}

	void SetAllData(TCellData value)
	{
		if (!m_Data)
			return;

		uint ttlSize = m_xSize * m_ySize;
		std::memset(m_Data, value, ttlSize);
	}

	// ##########################################################################################
	// UTILITY FUNCTIONS

	bool IsMapPosWithinBounds(TCellPos mapPos) const
	{
		return mapPos.x >= 0 && mapPos.x < static_cast<int>(m_xSize) &&
			mapPos.y >= 0 && mapPos.y < static_cast<int>(m_ySize);
	}

	// Returns position of cell center
	Vector2 PosMapToWorld(TCellPos mapPos) const
	{
		float halfres = 0.5f * m_Res_w;
		int dxMap = mapPos.x - m_xCenter;
		int dyMap = mapPos.y - m_yCenter;

		return Vector2(static_cast<float>(dxMap) * m_Res_w + halfres,
			static_cast<float>(dyMap) * m_Res_w + halfres);
	}

	Vector2 PosMapToWorldUnscaled(TCellPos mapPos) const
	{
		float halfCell = 0.5f;
		int dxMap = mapPos.x - m_xCenter;
		int dyMap = mapPos.y - m_yCenter;

		return Vector2(static_cast<float>(dxMap) + halfCell,
			static_cast<float>(dyMap) + halfCell);
	}

	TCellPos PosWorldToMap(Vector2 pos) const
	{
		return TCellPos(m_xCenter + static_cast<int>(floorf(pos.x / m_Res_w)),
			m_yCenter + static_cast<int>(floorf(pos.y / m_Res_w)));
	}

	TCellPos PosWorldUnscaledToMap(Vector2 pos_u) const
	{
		return TCellPos(m_xCenter + static_cast<int>(floorf(pos_u.x)),
			m_yCenter + static_cast<int>(floorf(pos_u.y)));
	}

	void GetWorldBounds(Vector2& outMins, Vector2& outMaxs) const
	{
		outMins.x = -m_Res_w * static_cast<float>(m_xCenter);
		outMins.y = -m_Res_w * static_cast<float>(m_yCenter);

		outMaxs.x = m_Res_w * static_cast<float>(m_xSize - m_xCenter);
		outMaxs.y = m_Res_w * static_cast<float>(m_ySize - m_yCenter);
	}

	// ##########################################################################################
	// PLAIN GETTERS
	// Returns size, in cells
	TCellSize GetSize() const { return TCellSize(m_xSize, m_ySize); }
	TCellSize GetCenter() const { return TCellSize(m_xCenter, m_yCenter); }
	float GetResolution_w() const { return m_Res_w; }

	// ##########################################################################################
	// MAP GROWTH
	// Ensures that given rectangle(in world coordinates) can be represented within map
	// Resizes the map accordingly

	void EnsureWorldBounds_Rect(Vector2 mins_w, Vector2 maxs_w)
	{
		ZoneScopedN("GridMap::EnsureWorldBounds");

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

	void EnsureWorldBounds_Circle(Vector2 center_w, float radius_w)
	{
		Vector2 mins_w(center_w.x - radius_w, center_w.y - radius_w);
		Vector2 maxs_w(center_w.x + radius_w, center_w.y + radius_w);
		EnsureWorldBounds_Rect(mins_w, maxs_w);
	}
};


//--------------------------------------------------------------------------------------------------------------------------
// Common map types used in project

// Occupancy map. !!! Must be signed! char is unsigned on ARM.
using OccupancyMap = GridMap<int8_t>;

// Distance map. We mostly care that value is above threshold, and values are clamped from upper side. uint8_t is enough.
using DistMap = GridMap<uint8_t>;

// Exploration map. Values are distance, and can be larger than 256, at least uint16_t is required.
using ExpMap = GridMap<uint16_t>;