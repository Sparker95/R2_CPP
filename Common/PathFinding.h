#pragma once

#include "Vector2.h"
#include "Map.h"

#include <stdint.h>
#include <vector>
#include <limits>
#include <algorithm>

namespace PathFinding
{
	using uint = unsigned int;

	namespace Transition
	{
		constexpr uint NONE  = 0; // Only for start node
		constexpr uint FWD   = 1;
		constexpr uint FWD_L = 2;
		constexpr uint BWD_L = 3;
		constexpr uint BWD   = 4;
		constexpr uint BWD_R = 5;
		constexpr uint FWD_R = 6;
	}

	struct Config
	{
		//--------------------------------------------------------------------------
		// Data
		// [*] - Updated in Init, not assumed to be filled in externally

		// Properties of transitions
		float m_TurnRadius = 0.6f;
		uint m_NumDirs     = 12;   // Number of directions.Direction index is 0..NumDirs - 1.
		float m_StepDist   = 0.1f; // [*] Distance of one step, always same, straight or in turn
		float m_StepDir    = 0.1f; // [*]

		// Car dimensions
		float m_CarWidth = 0.1f;
		float m_CarLengthFwd = 0.1f;
		float m_CarLengthBwd = 0.1f;
		float m_CarScaleNormal = 1.0f; // Some calls can switching between two size multipliers
		float m_CarScaleExtended = 1.0f;

		// Properties and costs and  of search
		float m_HeuristicMult = 1.0f;
		float m_OffStructureMult = 2.0f;  // Penalty of moving off structure map
		float m_StructureMapWidth = 0.1f; // Width of skeleton
		uint m_MaxNodes = 1024;

		// Multipliers of cost for each transition ID
		float m_MultFwd     = 1.0f;
		float m_MultTurnFwd = 1.0f;
		float m_MultBwd     = 1.0f;
		float m_MultTurnBwd = 1.0f;
		float m_TransIdMult[7]; // [*]

		// Direction vectors of each direction index
		std::vector<Vector2> m_DirVectors; // [*]

		//--------------------------------------------------------------------------

		// Calculates and fills remaining values
		void Init();

		const Vector2& GetDirVector(uint dir_id) { return m_DirVectors[dir_id]; }
	};

	struct Node
	{
		// Pose used in path finding, NOT Pose in the rest of code
		struct Pose
		{
			float x = 0;
			float y = 0;
			uint dir = 0;

			Pose() = default;
			Pose(float x, float y, uint dir) : x(x), y(y), dir(dir) {}
		};

		uint m_Id = -1; // Index of this node
		uint m_NodeFrom = -1;
		Node::Pose m_Pose;
		float m_F = 0;
		float m_G = 0;
		uint m_InTransId = Transition::NONE; // Transition TO this node

		Node() = default;

		Node(const Node& from, Node::Pose pose, uint trans)
		{
			m_NodeFrom = from.m_Id;
			m_Pose = pose;
			m_InTransId = trans;
		}
	};

	struct PathFindingResult
	{
		std::vector<Node> m_Nodes;
		uint m_NodesExplored = -1;
	};

	// Public functions

	// Some methods support parameter:
	// bool extendedCarScale - switches between multipliers of car size

	// Utils
	bool IsTransitionForward(uint transId); // True for forward transitions

	// Map calculations
	DistMap CalculateStructureMap(const DistMap& distMap, float structureWidth_w, float minDistToObstacle_w);

	// Calculates distance map from input map. Input map can be of different types.
	// threshold - input map cell is occupied if cell value is >= threshold value
	template<typename TInputMap>
	DistMap CalculateDistanceMap(const TInputMap& map, int threshold = 0)
	{
		ZoneScopedN("PathFinding::CalculateDistanceMap");

		const uint VALUE_MAX = std::numeric_limits<DistMap::TCellData>::max();

		auto mapCenter = map.GetCenter();
		auto mapSize = map.GetSize();
		DistMap distMap(mapSize.x, mapSize.y, mapCenter.x, mapCenter.y, map.GetResolution_w());

		// Same but signed
		Vector2Base<int> smapSize(static_cast<int>(mapSize.x), static_cast<int>(mapSize.y));

		for (int y = 0; y < smapSize.y; y++)
		{
			for (int x = 0; x < smapSize.x; x++)
			{
				if (static_cast<int>(static_cast<int>(map.GetDataAtPos(OccupancyMap::TCellPos(x, y)))) >= threshold)
					distMap.SetDataAtPos(OccupancyMap::TCellPos(x, y), 0);
				else
					distMap.SetDataAtPos(OccupancyMap::TCellPos(x, y), static_cast<DistMap::TCellData>(VALUE_MAX));
			}
		}

		// Note that we ignore border row and column completely!
		for (int j = 1; j < smapSize.y - 1; j++)
		{
			for (int i = 1; i < smapSize.x - 1; i++)
			{
				uint e[4] = {
					static_cast<uint>(distMap.GetDataAtPos({i,j - 1})) + 3,
					static_cast<uint>(distMap.GetDataAtPos({i - 1, j})) + 3,
					static_cast<uint>(distMap.GetDataAtPos({i - 1, j - 1})) + 4,
					static_cast<uint>(distMap.GetDataAtPos({i - 1, j + 1})) + 4 };

				for (uint f : e)
				{
					uint centerValue = static_cast<uint>(distMap.GetDataAtPos({ i, j }));
					if (centerValue > f)
						distMap.SetDataAtPos({ i, j }, std::clamp(f, 0u, VALUE_MAX));
				}
			}
		}

		for (int j = smapSize.y - 2; j > 0; j--)
		{
			for (int i = smapSize.x - 2; i > 0; i--)
			{
				uint e[4] = {
					static_cast<uint>(distMap.GetDataAtPos({i, j + 1})) + 3,
					static_cast<uint>(distMap.GetDataAtPos({i + 1, j})) + 3,
					static_cast<uint>(distMap.GetDataAtPos({i + 1, j - 1})) + 4,
					static_cast<uint>(distMap.GetDataAtPos({i + 1, j + 1})) + 4
				};

				for (uint f : e)
				{
					uint centerValue = static_cast<uint>(distMap.GetDataAtPos({ i, j }));
					if (centerValue > f)
						distMap.SetDataAtPos({ i, j }, std::clamp(f, 0u, VALUE_MAX));
				}
			}
		}

		return distMap;
	}

	// Calculates smooth arcs between nodes
	void CalculateSmoothTransition(const Config& config, const Node& nodeFrom, const Node& nodeTo, std::vector<Vector2>& outPositions, uint nIntervals, bool addLastPoint);

	// Can be used for calculation of start node pose
	Node::Pose CalculateNodePose(const Config& config, float x, float y, float dir_rad);

	// Checks if vehicle fits at position, returns true if fits
	bool CheckObstruction(const Config& config, const Vector2& pos, const Vector2& dir, const OccupancyMap& map, bool extendedCarScale);

	// The main path finding function
	void FindPath(const Config& config, const OccupancyMap& map, const Node::Pose& startPose, const Vector2& destPos, float destRadius, PathFindingResult& outResult, bool extendedCarScale);
}