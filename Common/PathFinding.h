#pragma once

#include "Vector2.h"

#include <stdint.h>
#include <vector>

class Map;

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
		uint m_TransId = Transition::NONE; // Transition TO this node

		Node() = default;

		Node(const Node& from, Node::Pose pose, uint trans)
		{
			m_NodeFrom = from.m_Id;
			m_Pose = pose;
			m_TransId = trans;
		}
	};

	struct PathFindingResult
	{
		std::vector<Node> m_Nodes;
		uint m_NodesExplored = -1;
	};

	// Public functions

	// Map calculations
	Map CalculateStructureMap(const Map& distMap, float structureWidth_w, float minDistToObstacle_w);
	Map CalculateDistanceMap(const Map& map, int threshold = 0);

	// Calculates smooth arcs between nodes
	void CalculateSmoothTransition(const Config& config, const Node& nodeFrom, const Node& nodeTo, std::vector<Vector2>& outPositions, uint nIntervals, bool addLastPoint);

	// Can be used for calculation of start node pose
	Node::Pose CalculateNodePose(const Config& config, float x, float y, float dir_rad);

	void FindPath(const Config& config, const Map& map, const Node::Pose& startPose, const Vector2& destPos, float destRadius, PathFindingResult& outResult);
}