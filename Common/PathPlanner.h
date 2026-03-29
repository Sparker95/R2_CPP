#pragma once

#include "Pose.h"
#include "Vector2.h"
#include "PathFinding.h"
#include "PathFollower.h"

class Map;
struct PathFollowerConfig;

// Plans path and stores the result in itself.
class PathPlanner
{
private:
	PathFinding::Config m_PFConfig;
	PathFollowerConfig m_FollowerConfig;
	std::vector<PathFollower::PathPoint> m_Path;

public:
	PathPlanner(const PathFinding::Config& pfConfig, const PathFollowerConfig& followerConfig);

	void PlanPath(const Map& map, const Pose& startPose, Vector2 destPos, float destRadius);
	void ResetPath();

	bool PathValid() const { return !m_Path.empty(); }
	const std::vector<PathFollower::PathPoint>& GetPath() const { return m_Path; }
};