#include "PathPlanner.h"
#include "PathFollower.h"
#include "TracyPlug.h"

#include <cstdio>

using uint = unsigned int;

PathPlanner::PathPlanner(const PathFinding::Config& pfConfig, const PathFollowerConfig& followerConfig)
{
	m_PFConfig = pfConfig;
	m_FollowerConfig = followerConfig;
}

void PathPlanner::PlanPath(const Map& map, const Pose& startPose, Vector2 destPos, float destRadius)
{
	ZoneScopedN("PathPlanner::CheckPathObstruction");

	// Reset the stored path either way
	ResetPath();

	PathFinding::Node::Pose startNodePose = PathFinding::CalculateNodePose(m_PFConfig,
		startPose.x, startPose.y, startPose.dir);

	printf("PathPlanner::PlanPath: %.3f %.3f %.3f -> %.3f %.3f r%.3f\n",
		startPose.x, startPose.y, startPose.dir,
		destPos.x, destPos.y, destRadius);

	PathFinding::PathFindingResult pfResult;
	// extendedCarScale = true - in path finding we use extended, bigger car size, to get path further from everything.
	PathFinding::FindPath(m_PFConfig, map, startNodePose, destPos, destRadius, pfResult, true);

	if (pfResult.m_Nodes.empty())
	{
		printf("PathPlanner::PlanPath: Failed to find path\n");
		return;
	}

	printf("PathPlanner::PlanPath: Found path: %u nodes, %u explored\n", (uint)pfResult.m_Nodes.size(), pfResult.m_NodesExplored);

	// Post processing of path
	// - Smoothing of arcs
	// - Extension points for path planner when direction changes
	const float extensionDistance = 3.0f * m_FollowerConfig.m_LookaheadDistance; // Must be several times larger
	std::vector<Vector2> tempSmoothPoints;
	for (uint i = 0; i < (uint)pfResult.m_Nodes.size()-1; i++)
	{
		const PathFinding::Node& node = pfResult.m_Nodes[i];
		const PathFinding::Node& nextNode = pfResult.m_Nodes[i + 1];

		bool nodeFwd = PathFinding::IsTransitionForward(node.m_InTransId);
		bool nextNodeFwd = PathFinding::IsTransitionForward(nextNode.m_InTransId);

		bool addExtension = nodeFwd != nextNodeFwd; // Change of direciton

		if (addExtension)
		{
			// Last added point must be marked as end point
			if (!m_Path.empty())
				m_Path.rbegin()->m_Type = PathFollower::PathPoint::Type::MAIN_END;

			const Vector2& nodeDirVector = m_PFConfig.m_DirVectors[node.m_Pose.dir];
			float extensionDirSign = nodeFwd ? 1.0f : -1.0f;

			Vector2 extensionPointPos = Vector2(node.m_Pose.x, node.m_Pose.y) + nodeDirVector * (extensionDistance * extensionDirSign);
			PathFollower::PathPoint fpp;
			fpp.m_Pos = extensionPointPos;
			fpp.m_OutMoveFwd = nextNodeFwd; // Same as main node
			fpp.m_Type = PathFollower::PathPoint::Type::EXTENSION;
			m_Path.push_back(fpp);
		}

		PathFinding::CalculateSmoothTransition(m_PFConfig, node, nextNode,
			tempSmoothPoints, 3, true);

		for (uint smoothPtId = 0; smoothPtId < (uint)tempSmoothPoints.size(); smoothPtId++)
		{
			const auto& smoothPoint = tempSmoothPoints[smoothPtId];

			// Skip same points
			if (smoothPtId == 0 && !m_Path.empty())
			{
				auto lastAddedPoint = m_Path.rbegin();
				if (lastAddedPoint->m_Pos == smoothPoint)
					continue;
			}

			PathFollower::PathPoint fpp;
			fpp.m_Pos = smoothPoint;
			fpp.m_OutMoveFwd = nextNodeFwd; // ! We use fwd value TO next node

			// 'Patch' the first and last inserted PathPoint: set type, copy direction
			if (smoothPtId == 0)
			{
				fpp.m_Type = PathFollower::PathPoint::Type::MAIN;
				fpp.m_Dir = m_PFConfig.GetDirVector(node.m_Pose.dir);
			}
			else if (smoothPtId == (uint)tempSmoothPoints.size() - 1)
			{
				fpp.m_Type = PathFollower::PathPoint::Type::MAIN;
				fpp.m_Dir = m_PFConfig.GetDirVector(nextNode.m_Pose.dir);
			}
			else
				fpp.m_Type = PathFollower::PathPoint::Type::MAIN_SMOOTH;

			m_Path.push_back(fpp);
		}
	}

	// Special handling for last node. Extend it too.
	{
		const PathFinding::Node& lastNode = *pfResult.m_Nodes.rbegin();
		bool lastNodeFwd = PathFinding::IsTransitionForward(lastNode.m_InTransId);

		if (!m_Path.empty())
			m_Path.rbegin()->m_Type = PathFollower::PathPoint::Type::MAIN_END;

		const Vector2& nodeDirVector = m_PFConfig.m_DirVectors[lastNode.m_Pose.dir];
		float extensionDirSign = lastNodeFwd ? 1.0f : -1.0f;

		Vector2 extensionPointPos = Vector2(lastNode.m_Pose.x, lastNode.m_Pose.y) + nodeDirVector * (extensionDistance * extensionDirSign);
		PathFollower::PathPoint fpp;
		fpp.m_Pos = extensionPointPos;
		fpp.m_OutMoveFwd = lastNodeFwd; // Same direction as we came here
		fpp.m_Type = PathFollower::PathPoint::Type::EXTENSION;
		m_Path.push_back(fpp);
	}

	// Calculate m_DistFromStart for each point
	m_Path[0].m_DistFromStart = 0;
	for (uint i = 1; i < (uint)m_Path.size(); i++)
	{
		auto& prevPt = m_Path[i - 1];
		auto& thisPt = m_Path[i];
		thisPt.m_DistFromStart = prevPt.m_DistFromStart + Vector2::Distance(prevPt.m_Pos, thisPt.m_Pos);
	}
}

bool PathPlanner::CheckPathObstruction(const Map& map, const std::vector<PathFollower::PathPoint>& path, uint currentPathPointId, uint* pOutObstrPointId)
{
	ZoneScopedN("PathPlanner::CheckPathObstruction");

	for (uint pointId = currentPathPointId; pointId < (uint)path.size(); pointId++)
	{
		const PathFollower::PathPoint& point = path[pointId];
		using TPathPointType = PathFollower::PathPoint::Type;

		// Obstruction can only be checked at MAIN and MAIN_END points,
		// because path obstruction was checked only at those points.
		if (point.m_Dir != Vector2::ZERO && (point.m_Type == TPathPointType::MAIN || point.m_Type == TPathPointType::MAIN_END))
		{
			// extendedCarScale = false - in path obstruction checks we use normal car size
			if (!PathFinding::CheckObstruction(m_PFConfig, point.m_Pos, point.m_Dir, map, false))
			{
				if (pOutObstrPointId)
					*pOutObstrPointId = pointId;
				return false;
			}
		}
	}
	return true;
}

void PathPlanner::ResetPath()
{
	m_Path.clear();
}