#include "PathFinding.h"

#include "MathUtils.h"
#include "Map.h"
#include "LineRasterizer.h"
#include "TracyPlug.h"
#include "Utils.h"

#include <cmath>
#include <unordered_map>
#include <cstdio>
#include <limits>

using namespace PathFinding;

using Vector2 = ::Vector2;
using Node = PathFinding::Node;

void PathFinding::Config::Init()
{
	m_StepDist = PI_2X / static_cast<float>(m_NumDirs) * m_TurnRadius;
	m_StepDir = PI_2X / static_cast<float>(m_NumDirs);

	m_TransIdMult[(int)Transition::NONE] = 1.0f;
	m_TransIdMult[(int)Transition::FWD] = m_MultFwd;
	m_TransIdMult[(int)Transition::FWD_L] = m_MultTurnFwd;
	m_TransIdMult[(int)Transition::BWD_L] = m_MultTurnBwd;
	m_TransIdMult[(int)Transition::BWD] = m_MultBwd;
	m_TransIdMult[(int)Transition::BWD_R] = m_MultTurnBwd;
	m_TransIdMult[(int)Transition::FWD_R] = m_MultTurnFwd;

	m_DirVectors.resize(m_NumDirs);
	for (uint i = 0; i < m_NumDirs; i++)
	{
		float dir_rad = PI_2X / static_cast<float>(m_NumDirs) * static_cast<float>(i);
		m_DirVectors[i].x = std::cos(dir_rad);
		m_DirVectors[i].y = std::sin(dir_rad);
	}
}

// #https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
Vector2 RotatePoint(const Vector2& pivot, float angle_rad, const Vector2& pt)
{
	float s = std::sin(angle_rad);
	float c = std::cos(angle_rad);

	// translate point back to origin :
	float cx = pivot.x;
	float cy = pivot.y;
	float x = pt.x - cx;
	float y = pt.y - cy;

	// rotate point
	float xnew = x * c - y * s;
	float ynew = x * s + y * c;

	// translate point back :
	return Vector2(xnew + cx, ynew + cy);
}

void FindNeighborNodes(const Config& config, const Node& node, std::vector<Node>& outNodes)
{
	outNodes.clear();

	Vector2 currentPos(node.m_Pose.x, node.m_Pose.y);
	uint dir_id = node.m_Pose.dir;

	const Vector2& vDir = config.m_DirVectors[dir_id]; //  Vector of current direction
	uint transId = node.m_InTransId;

	// First trivial cases - move straight fwd or backward
	float stepDist = config.m_StepDist;
	if (transId != Transition::BWD)
	{
		Node newNode(node, Node::Pose(currentPos.x + stepDist * vDir.x, currentPos.y + stepDist * vDir.y, dir_id), Transition::FWD);
		outNodes.push_back(newNode);
	}
	if (transId != Transition::FWD)
	{
		Node newNode(node, Node::Pose(currentPos.x - stepDist * vDir.x, currentPos.y - stepDist * vDir.y, dir_id), Transition::BWD);
		outNodes.push_back(newNode);
	}

	Vector2 vSide(vDir.y, -vDir.x);
	float turnRadius = config.m_TurnRadius;

	// !!! Be careful with % operator, it is problematic with negative numbers. In Python it behaves different.

	// Rotate around right point
	Vector2 rightTurnPt(currentPos.x + turnRadius * vSide.x, currentPos.y + turnRadius * vSide.y);
	if (transId != Transition::BWD_R)
	{
		Vector2 ptRtFwd = RotatePoint(rightTurnPt, -config.m_StepDir, currentPos);
		Node newNode(node, Node::Pose(ptRtFwd.x, ptRtFwd.y, (dir_id - 1 + config.m_NumDirs) % config.m_NumDirs), Transition::FWD_R);
		outNodes.push_back(newNode);
	}
	if (transId != Transition::FWD_R)
	{
		Vector2 ptRtBwd = RotatePoint(rightTurnPt, config.m_StepDir, currentPos);
		Node newNode(node, Node::Pose(ptRtBwd.x, ptRtBwd.y, (dir_id + 1) % config.m_NumDirs), Transition::BWD_R);
		outNodes.push_back(newNode);
	}

	// Rotate around left point
	Vector2 leftTurnPt(currentPos.x - turnRadius * vSide.x, currentPos.y - turnRadius * vSide.y);
	if (transId != Transition::BWD_L)
	{
		Vector2 ptLtFwd = RotatePoint(leftTurnPt, config.m_StepDir, currentPos);
		Node newNode(node, Node::Pose(ptLtFwd.x, ptLtFwd.y, (dir_id + 1) % config.m_NumDirs), Transition::FWD_L);
		outNodes.push_back(newNode);
	}
	if (transId != Transition::FWD_L)
	{
		Vector2 ptLtBwd = RotatePoint(leftTurnPt, -config.m_StepDir, currentPos);
		Node newNode(node, Node::Pose(ptLtBwd.x, ptLtBwd.y, (dir_id - 1 + config.m_NumDirs) % config.m_NumDirs), Transition::BWD_L);
		outNodes.push_back(newNode);
	}
}

float Heuristic(const Config& config, const Node::Pose& thisPose, const Node::Pose& otherPose)
{
	// Direction is irrelevant! Function accepts poses for consistency.
	float dx = otherPose.x - thisPose.x;
	float dy = otherPose.y - thisPose.y;
	return config.m_HeuristicMult * std::sqrt(dx * dx + dy * dy);
}

// Calculates direction index from direction in radians
uint CalculateDirectionId(float dir_rad, uint numDirs)
{
	float step_dir = PI_2X / static_cast<float>(numDirs);
	uint dir_id = static_cast<uint>(std::round(Utils::WrapAngleFull(dir_rad) / step_dir)) % numDirs;
	return dir_id;
}

DistMap PathFinding::CalculateStructureMap(const DistMap& distMap, float structureWidth_w, float minDistToObstacle_w)
{
	auto mapCenter = distMap.GetCenter();
	auto mapSize = distMap.GetSize();
	DistMap structureMap(mapSize.x, mapSize.y, mapCenter.x, mapCenter.y, distMap.GetResolution_w());
	Vector2Base<int> smapSize(static_cast<int>(mapSize.x), static_cast<int>(mapSize.y));

	Vector2Base<int> offsetPairs[4][2] = {
		{{-1,  0}, {1, 0}},
		{{0,  -1}, {0, 1}},
		{{-1, -1}, {1, 1}},
		{{-1,  1}, {1, -1}}
	};

	for (int y = 1; y < smapSize.y - 1; y++)
	{
		for (int x = 1; x < smapSize.x - 1; x++)
		{
			bool ridge = false;
			uint centerValue = static_cast<uint>(distMap.GetDataAtPos({ x, y }));
			for (int pairId = 0; pairId < 4; pairId++)
			{
				uint v0 = static_cast<uint>(distMap.GetDataAtPos({
					x + offsetPairs[pairId][0].x,
					y + offsetPairs[pairId][0].y }));
				uint v1 = static_cast<uint>(distMap.GetDataAtPos({
					x + offsetPairs[pairId][1].x,
					y + offsetPairs[pairId][1].y
					}));

				if (centerValue > v0 && centerValue > v1)
				{
					ridge = true;
					break;
				}
			}

			if (ridge && centerValue != 0)
				structureMap.SetDataAtPos({ x, y }, 1);
			else
				structureMap.SetDataAtPos({ x, y }, 0);
		}
	}

	// Make structure wider
	DistMap structureDistMap = CalculateDistanceMap(structureMap, 1);

	uint structureDistMapTreshold = static_cast<uint>(std::ceil(0.5f * structureWidth_w / distMap.GetResolution_w())) * 3;
	uint distToObstacleThreshold = static_cast<uint>(std::floor(minDistToObstacle_w / distMap.GetResolution_w())) * 3;

	for (int y = 0; y < smapSize.y; y++)
	{
		for (int x = 0; x < smapSize.x; x++)
		{
			if (static_cast<uint>(structureDistMap.GetDataAtPos({ x, y })) < structureDistMapTreshold && // Close to structure
				static_cast<uint>(distMap.GetDataAtPos({ x, y })) > distToObstacleThreshold) // Not too close to obstacle
				structureDistMap.SetDataAtPos({ x, y }, 1);
			else
				structureDistMap.SetDataAtPos({ x, y }, 0);
		}
	}

	return structureDistMap;
}

float PathFinding::DistanceMapValueToWorld(DistMap::TCellData distMapValue, float mapRes_w)
{
	return static_cast<float>(static_cast<uint>(distMapValue)) / 3.0f * mapRes_w;
}

float PathFinding::WorldToDistanceMapValue(float worldValue, float mapRes_w)
{
	return 3.0f * (worldValue / mapRes_w);
}

ECheckObstruction PathFinding::CheckObstruction(const Config& config, const Vector2& pos, const Vector2& dir, const OccupancyMap& map, bool extendedCarScale)
{
	float mapRes_w = map.GetResolution_w();
	Vector2 pos_u(pos.x / mapRes_w, pos.y / mapRes_w); // In unscaled coordinates(not world coordinates)

	OccupancyMap::TCellPos mapPos = map.PosWorldUnscaledToMap({ pos_u.x, pos_u.y });

	if (!map.IsMapPosWithinBounds(mapPos))
		return ECheckObstruction::HIT_OCCUPIED_CELLS;

	// Check if this position is occupied
	if (map.GetDataAtPos(mapPos) > 0)
		return ECheckObstruction::HIT_OCCUPIED_CELLS;

	float carScale = extendedCarScale ? config.m_CarScaleExtended : config.m_CarScaleNormal;

	float offsetFwd_u = 0.5f * (config.m_CarLengthFwd - config.m_CarLengthBwd) / mapRes_w;
	Vector2 rectCenterPos_u(pos_u.x + offsetFwd_u * dir.x, pos_u.y + offsetFwd_u * dir.y);

	std::vector<LineRasterizer::HSpan> rectSpans;
	Vector2 rectSize(config.m_CarWidth / mapRes_w, (config.m_CarLengthFwd + config.m_CarLengthBwd) / mapRes_w);
	rectSize *= carScale;
	LineRasterizer::RasterizeRect(rectCenterPos_u, rectSize, { dir.x, dir.y }, rectSpans);

	// Iterate spans of rectangle
	OccupancyMap::TCellSize mapCenter = map.GetCenter();
	uint nUnknownCells = 0;
	for (const auto& span : rectSpans)
	{
		int y_map = span.y + mapCenter.y; // Indices are in map's native form
		int xStart_map = span.xStart + mapCenter.x;
		int xEnd_map = span.xEnd + mapCenter.x;

		// This is a single line (row) with start and end x coordinate, and fixed y coordinate
		// If either end doesn't fit, we're out of map

		if (!map.IsMapPosWithinBounds({ xStart_map, y_map }))
			return ECheckObstruction::HIT_OCCUPIED_CELLS;

		if (!map.IsMapPosWithinBounds({ xEnd_map, y_map }))
			return ECheckObstruction::HIT_OCCUPIED_CELLS;

		// This line span is inside map, check individual cells
		for (int x_map = xStart_map; x_map <= xEnd_map; x_map++)
		{
			auto cellValue = map.GetDataAtPos({ x_map, y_map });
			if (cellValue > 0)
				return ECheckObstruction::HIT_OCCUPIED_CELLS;
			else if (cellValue == 0)
				nUnknownCells++;
		}
	}

	if (nUnknownCells != 0)
		return ECheckObstruction::HIT_UNKNOWN_CELLS;
	else
		return ECheckObstruction::FREE;
}

void RestorePath(const Node& endNode, const std::vector<Node>& allNodes, std::vector<Node>& outNodes)
{
	const Node* pCurrentNode = &endNode;

	outNodes.clear();
	while (true)
	{
		outNodes.insert(outNodes.begin(), *pCurrentNode);
		if (pCurrentNode->m_NodeFrom == static_cast<uint>(-1))
			break;
		pCurrentNode = &allNodes[pCurrentNode->m_NodeFrom];
	}
}

//------------------------------------------------------------------------------------------------------------------------------
// Public functions

bool PathFinding::IsTransitionForward(uint transId)
{
	return transId == Transition::NONE ||
		transId == Transition::FWD ||
		transId == Transition::FWD_L ||
		transId == Transition::FWD_R;
}

void PathFinding::CalculateSmoothTransition(const Config& config, const Node& nodeFrom, const Node& nodeTo,
	std::vector<Vector2>& outPositions, uint nIntervals, bool addLastPoint)
{
	uint transId = nodeTo.m_InTransId;

	// Trivial case, fwd or bwd, return just pair of points without smoothing
	if (transId == Transition::FWD || transId == Transition::BWD)
	{
		outPositions.clear();
		outPositions.push_back(Vector2(nodeFrom.m_Pose.x, nodeFrom.m_Pose.y));
		if (addLastPoint)
			outPositions.push_back(Vector2(nodeTo.m_Pose.x, nodeTo.m_Pose.y));
		return;
	}

	// Non trivial cases
	const Vector2& vDir = config.m_DirVectors[nodeFrom.m_Pose.dir];
	Vector2 vSide(vDir.y, -vDir.x);
	float turnRadius = config.m_TurnRadius;
	float turnPointSign = (transId == Transition::FWD_R || transId == Transition::BWD_R) ? 1.0f : -1.0f;

	Vector2 turnPoint(nodeFrom.m_Pose.x + turnPointSign * turnRadius * vSide.x, nodeFrom.m_Pose.y + turnPointSign * turnRadius * vSide.y);
	Vector2 turnFromPos(nodeFrom.m_Pose.x, nodeFrom.m_Pose.y);

	float turnAngleSign = (transId == Transition::FWD_L || transId == Transition::BWD_R) ? 1.0f : -1.0f;

	uint nOutPoints = nIntervals;
	if (addLastPoint)
		nOutPoints++;
	outPositions.resize(nOutPoints);

	// !!! First and last point must be exactly same as node positions. Path Planner relies on that.

	// First point
	outPositions[0] = turnFromPos;

	// Intermetidate points
	for (uint i = 1; i < nIntervals; i++)
	{
		float turnAngle_rad = static_cast<float>(i) * turnAngleSign * config.m_StepDir / static_cast<float>(nIntervals);
		Vector2 smoothPos = RotatePoint(turnPoint, turnAngle_rad, turnFromPos);
		outPositions[i] = smoothPos;
	}

	// Last point
	if (addLastPoint)
		outPositions[nOutPoints - 1] = Vector2(nodeTo.m_Pose.x, nodeTo.m_Pose.y);
}

Node::Pose PathFinding::CalculateNodePose(const Config& config, float x, float y, float dir_rad)
{
	Node::Pose p;
	p.x = x;
	p.y = y;
	p.dir = CalculateDirectionId(dir_rad, config.m_NumDirs);
	return p;
}

struct PoseHashKey
{
	int x;
	int y;
	uint dir;

	PoseHashKey() = default;

	bool operator==(const PoseHashKey& other) const noexcept
	{
		return x == other.x && y == other.y && dir == other.dir;
	}

	static PoseHashKey InitFromNode(const Node& node, const OccupancyMap& map)
	{
		PoseHashKey k;
		OccupancyMap::TCellPos mapPos = map.PosWorldToMap({ node.m_Pose.x, node.m_Pose.y });
		k.x = mapPos.x;
		k.y = mapPos.y;
		k.dir = node.m_Pose.dir;
		return k;
	}
};

template<>
struct std::hash<PoseHashKey>
{
	std::size_t operator()(const PoseHashKey& hk) const noexcept
	{
		return static_cast<std::size_t>(hk.x & 0xFFFF) |
			(static_cast<std::size_t>(hk.y & 0xFFFF) << 16) |
			(static_cast<std::size_t>(hk.dir & 0xFF) << 32);
	}
};

void PathFinding::FindPath(const Config& config, const OccupancyMap& map, const Node::Pose& startPose, const Vector2& destPos, float destRadius, PathFindingResult& outResult, bool extendedCarScale)
{
	ZoneScopedN("PathFinding::FindPath");

	DistMap distMap = CalculateDistanceMap(map);
	DistMap structureMap = CalculateStructureMap(distMap, config.m_StructureMapWidth, 0.5f * config.m_CarWidth);

	Node startNode;
	Node::Pose destPose(destPos.x, destPos.y, 0);
	startNode.m_Pose = startPose;
	startNode.m_F = Heuristic(config, startPose, destPose);
	startNode.m_Id = 0;

	std::vector<Node> allNodes;
	std::vector<uint> openSet;

	allNodes.push_back(startNode);
	openSet.push_back(0);

	std::unordered_map<PoseHashKey, uint> nodeHashMap; // Key - cell position & dir, value - node index
	nodeHashMap.insert({ PoseHashKey::InitFromNode(startNode, map), 0 });

	std::vector<Node> neighborNodes;
	neighborNodes.reserve(6);

	while (!openSet.empty() && allNodes.size() < config.m_MaxNodes)
	{
		// Find lowest F score node
		uint minFScoreNodeId = openSet[0]; // Index of lowest F score node in allNodes
		uint minFScoreOpenSetId = 0; // Index of element in open set, pointing at that node
		float minFScore = allNodes[openSet[0]].m_F;
		for (uint i = 0, ossize = (uint)openSet.size(); i < ossize; i++)
		{
			uint nodeId = openSet[i];
			if (allNodes[nodeId].m_F < minFScore)
			{
				minFScore = allNodes[nodeId].m_F;
				minFScoreNodeId = nodeId;
				minFScoreOpenSetId = i;
			}
		}

		uint currentNodeId = minFScoreNodeId; // Must always acces by index, because array grows
		openSet.erase(openSet.begin() + minFScoreOpenSetId);

		// Check collision at this node, skip it if car can't be here
		ECheckObstruction obstruction = CheckObstruction(config,
			{ allNodes[currentNodeId].m_Pose.x, allNodes[currentNodeId].m_Pose.y },
			config.m_DirVectors[allNodes[currentNodeId].m_Pose.dir],
			map, extendedCarScale);
		if (obstruction == ECheckObstruction::HIT_OCCUPIED_CELLS)
			continue;

		// We have arrived
		float dx = allNodes[currentNodeId].m_Pose.x - destPos.x;
		float dy = allNodes[currentNodeId].m_Pose.y - destPos.y;
		if (dx * dx + dy * dy < destRadius * destRadius)
		{
			RestorePath(allNodes[currentNodeId], allNodes, outResult.m_Nodes);
			outResult.m_NodesExplored = static_cast<uint>(allNodes.size());
			return;
		}

		FindNeighborNodes(config, allNodes[currentNodeId], neighborNodes);
		for (Node& neighborNode : neighborNodes)
		{
			OccupancyMap::TCellPos neighborNodeMapPos = map.PosWorldToMap({ neighborNode.m_Pose.x, neighborNode.m_Pose.y });

			// Instantly check if it's out of bounds or is instantly in obstacle
			if (!map.IsMapPosWithinBounds(neighborNodeMapPos))
				continue;
			if (map.GetDataAtPos(neighborNodeMapPos) > 0)
				continue;

			// However don't do full collision test yet

			float edgeWeight = 1.0f;
			edgeWeight *= config.m_TransIdMult[neighborNode.m_InTransId];
			edgeWeight *= config.m_StepDist;
			if (structureMap.GetDataAtPos(neighborNodeMapPos) != 1) // If we step out of structure
				edgeWeight *= config.m_OffStructureMult;
			if (obstruction == ECheckObstruction::HIT_UNKNOWN_CELLS)
				edgeWeight *= config.m_HitUnknownCellsMult;

			float neighborNodeG = allNodes[currentNodeId].m_G + edgeWeight;

			PoseHashKey poseHashKey = { neighborNodeMapPos.x, neighborNodeMapPos.y, neighborNode.m_Pose.dir };

			auto nodeIdFromDictIt = nodeHashMap.find(poseHashKey);

			if (nodeIdFromDictIt != nodeHashMap.end())
			{
				// We have already been there
				uint nodeFromDictId = (*nodeIdFromDictIt).second;
				Node& nodeFromDict = allNodes[nodeFromDictId];

				if (nodeFromDict.m_Pose.dir != neighborNode.m_Pose.dir)
					throw std::exception();

				if (nodeFromDict.m_G > neighborNodeG)
				{
					nodeFromDict.m_G = neighborNodeG;
					nodeFromDict.m_F = neighborNodeG + Heuristic(config, neighborNode.m_Pose, destPose);
					nodeFromDict.m_NodeFrom = currentNodeId;
					nodeFromDict.m_InTransId = neighborNode.m_InTransId;
					// Do NOT add to allNode and openSet
				}
			}
			else
			{
				// We have never been there
				neighborNode.m_G = neighborNodeG;
				neighborNode.m_F = neighborNodeG + Heuristic(config, neighborNode.m_Pose, destPose);
				neighborNode.m_Id = static_cast<uint>(allNodes.size());

				// Add to allNodes and openSet and nodeHashMap
				allNodes.push_back(neighborNode);
				openSet.push_back(neighborNode.m_Id);
				nodeHashMap.insert({ poseHashKey, neighborNode.m_Id });
			}
		}
	}

	// If we are here, we failed to find path
	return;
}

void PathFinding::FindUnobstructPath(const Config& config, const OccupancyMap& map, const Node::Pose& startPose, PathFindingResult& outResult, bool extendedCarScale)
{
	ZoneScopedN("PathFinding::FindUnobstructPath");

	// Now the function doesn't do a proper BFS, but simply considers one move.

	Node startNode;
	startNode.m_Pose = startPose;
	startNode.m_F = 0;
	startNode.m_Id = 0;

	std::vector<Node> allNodes;
	allNodes.push_back(startNode);

	std::vector<Node> neighborNodes;
	neighborNodes.reserve(6);

	FindNeighborNodes(config, startNode, neighborNodes);

	for (Node& neighborNode : neighborNodes)
	{
		ECheckObstruction obstruction = CheckObstruction(config,
			{ neighborNode.m_Pose.x, neighborNode.m_Pose.y },
			config.m_DirVectors[neighborNode.m_Pose.dir],
			map, extendedCarScale);

		if (obstruction == ECheckObstruction::HIT_OCCUPIED_CELLS)
			neighborNode.m_NodeFrom = -1; // Mark it invalid
		else
		{
			float edgeWeight = 1.0f;
			edgeWeight *= config.m_TransIdMult[neighborNode.m_InTransId];
			edgeWeight *= config.m_StepDist;
			neighborNode.m_G = edgeWeight; 
		}
	}

	// Find node with best score
	float minScore = std::numeric_limits<float>::max();
	const Node* pMinScoreNode = nullptr;
	for (Node& neighborNode : neighborNodes)
	{
		if (neighborNode.m_NodeFrom == -1)
			continue;
		if (neighborNode.m_G < minScore)
		{
			minScore = neighborNode.m_G;
			pMinScoreNode = &neighborNode;
		}
	}

	// No valid movement found -> failure
	if (pMinScoreNode == nullptr)
		return;

	// Restore path
	RestorePath(*pMinScoreNode, allNodes, outResult.m_Nodes);
	outResult.m_NodesExplored = 1; // Diagnostics data, not important
}