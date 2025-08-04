#pragma once

#include "Common.h"
#include "Vector2.h"

class Map
{
public:
	using TCellData = unsigned char;
	constexpr static TCellData CELL_VALUE_UNKNOWN = 127;
	constexpr static TCellData CELL_VALUE_MAX = 255;

	// Map's native position format
	using CellPos = Vector2Base<int>;

private:

	TCellData* m_Data = nullptr; // todo might want to split m_Data & m_DataSize into own class
	uint m_DataSize = 0;
	float m_ResWorld = 0;
	int m_Size = 0;

public:

	Map() = default;
	Map(int mapSize); // mapSize - number of cells, width and height
	Map(const Map& other) noexcept; // Copy
	Map& operator=(const Map& other) noexcept;
	Map(Map&& other) noexcept; // Move
	~Map();

	// Getters
	const TCellData* GetData() const { return m_Data; }
	TCellData* GetData() { return m_Data; }
	TCellData GetCellData(CellPos mapPos) const { return m_Data[m_Size*mapPos.y + mapPos.x]; }
	int GetSize() const { return m_Size; }

	// Modificaiton of map data
	void SetCellData(CellPos mapPos, TCellData value);
	void SetAllCellsData(TCellData value);

	// Utility functions
	bool IsCellPosWithinBounds(CellPos mapPos) const;
	static Vector2 GetCellCenterPos(CellPos cellPos); // Returns position of cell center
	static CellPos GetCellIndex(Vector2 pos);

private:
	void CopyData(TCellData* to, const TCellData* from, uint size);
};