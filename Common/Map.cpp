#include "Map.h"

#include <cmath>
#include "Vector2.h"

Map::Map(int mapSideSizeCells) :
	m_Size(mapSideSizeCells)
{
	m_Data = new TCellData[m_Size * m_Size];

	// Init data
	m_DataSize = m_Size * m_Size;
	for (uint i = 0; i < m_DataSize; i++)
		m_Data[i] = CELL_VALUE_UNKNOWN;
}

// Copy
Map::Map(const Map& other) noexcept
{
	m_Size = other.m_Size;

	m_Data = new TCellData[m_Size * m_Size];
	m_DataSize = other.m_DataSize;

	CopyData(m_Data, other.m_Data, m_DataSize);
}
// Assignment
Map& Map::operator=(const Map& other) noexcept
{
	if (m_Data)
	{
		if (m_Size != other.m_Size)
		{
			delete[] m_Data;
			m_Data = nullptr;
		}
	}

	m_Size = other.m_Size;

	if (!m_Data)
		m_Data = new TCellData[m_Size * m_Size];
	
	m_DataSize = other.m_DataSize;

	CopyData(m_Data, other.m_Data, m_DataSize);
	return *this;
}

// Move
Map::Map(Map&& other) noexcept
{
	m_Size = other.m_Size;
	
	m_Data = other.m_Data;
	other.m_Data = nullptr;
}

void Map::CopyData(TCellData* to, const TCellData* from, uint size)
{
	// todo replace with memcpy
	for (uint i = 0; i < size; i++)
		to[i] = from[i];
}

Map::~Map()
{
	if (m_Data)
		delete[] m_Data;
}

void Map::SetCellData(CellPos mapPos, TCellData value)
{
	m_Data[m_Size * mapPos.y + mapPos.x] = value;
}

void Map::SetAllCellsData(TCellData value)
{
	for (int x = 0; x < m_Size; x++)
	{
		for (int y = 0; y < m_Size; y++)
		{
			SetCellData(CellPos(x, y), value);
		}
	}
}

Vector2 Map::GetCellCenterPos(CellPos mapPos)
{
	constexpr float halfCell = 0.5f;
	return Vector2(static_cast<float>(mapPos.x) + halfCell, static_cast<float>(mapPos.y) + halfCell);
}

Map::CellPos Map::GetCellIndex(Vector2 pos)
{
	return CellPos(static_cast<int>(floorf(pos.x)), static_cast<int>(floorf(pos.y)));
}

bool Map::IsCellPosWithinBounds(CellPos mapPos) const
{
	return mapPos.x >= 0 && mapPos.x < m_Size && mapPos.y >= 0 && mapPos.y < m_Size;
}
