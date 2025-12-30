#pragma once

#include "CommonIntTypes.h"
#include "Vector2.h"

#include <cstdint>

class Map
{
public:
	using TCellData = std::int8_t; // !!! Must be signed! char is unsigned on ARM.	
	using TCellPos = Vector2Base<int>;  // Map's native position type. Position can be negative.
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

	Map() = default;
	Map(float resolution_w);
	Map(const Map& other) noexcept; // Copy
	Map& operator=(const Map& other) noexcept;
	Map(Map&& other) noexcept; // Move
	~Map();

	// Accessing data
	TCellData* GetData() { return m_Data; }
	TCellData GetDataAtPos(TCellPos mapPos) const;
	void SetDataAtPos(TCellPos mapPos, TCellData value);
	void SetAllData(TCellData value);

	// Utility functions
	bool IsMapPosWithinBounds(TCellPos mapPos) const;
	Vector2 PosMapToWorld(TCellPos mapPos) const; // Returns position of cell center
	Vector2 PosMapToWorldUnscaled(TCellPos mapPos) const;
	TCellPos PosWorldToMap(Vector2 pos) const;
	TCellPos PosWorldUnscaledToMap(Vector2 pos_u) const;
	void GetWorldBounds(Vector2& outMins, Vector2& outMaxs) const;

	// Plain getters
	// Returns size, in cells
	TCellSize GetSize() const { return TCellSize(m_xSize, m_ySize); }
	TCellSize GetCenter() const { return TCellSize(m_xCenter, m_yCenter); }
	float GetResolution_w() const { return m_Res_w; }

	// Map growth
	// Ensures that given rectangle(in world coordinates) can be represented within map
	// Resizes the map accordingly
	void EnsureWorldBounds_Rect(Vector2 mins_w, Vector2 maxs_w);
	void EnsureWorldBounds_Circle(Vector2 center_w, float radius_w);
};