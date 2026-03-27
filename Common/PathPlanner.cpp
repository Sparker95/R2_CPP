#include "PathPlanner.h"

using uint = unsigned int;

PathPlanner::PathPlanner(const PathFinding::Config& pfConfig)
{
	m_PFConfig = pfConfig;
}

void PathPlanner::PlanPath(const Map& map, const Pose& startPose, Vector2 destPos, float destRadius)
{
	// Reset the stored path either way
	ResetPath();

	PathFinding::Node::Pose startNodePose = PathFinding::CalculateNodePose(m_PFConfig,
		startPose.x, startPose.y, startPose.dir);

	PathFinding::PathFindingResult pfResult;
	PathFinding::FindPath(m_PFConfig, map, startNodePose, destPos, destRadius, pfResult);

	if (pfResult.m_Nodes.empty())
	{
		return;
	}

	// WIP test
	//m_SmoothPath.clear();
	//for (const auto& node : pfResult.m_Nodes)
	//	m_SmoothPath.push_back({node.m_Pose.x, node.m_Pose.y});

	// Process the resulting node points: calculate intermediate points to achieve smother arcs
	m_SmoothPath.clear();
	std::vector<Vector2> tempTransitionPoints;
	for (uint i = 0; i < (uint)pfResult.m_Nodes.size()-1; i++)
	{
		const PathFinding::Node& node = pfResult.m_Nodes[i];
		const PathFinding::Node& nextNode = pfResult.m_Nodes[i + 1];

		bool addLastPoint = i == (uint)pfResult.m_Nodes.size() - 2; // Last point
		PathFinding::CalculateSmoothTransition(m_PFConfig, node, nextNode,
			tempTransitionPoints, 3, addLastPoint);
		for (const auto& p : tempTransitionPoints)
			m_SmoothPath.push_back(p);
	}
}

void PathPlanner::ResetPath()
{
	m_SmoothPath.clear();
}