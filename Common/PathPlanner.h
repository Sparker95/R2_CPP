#pragma once

#include "Pose.h"
#include "Vector2.h"
#include "PathFinding.h"

class Map;

// Plans path and stores the result in itself.
class PathPlanner
{
private:
	PathFinding::Config m_PFConfig;
	std::vector<Vector2> m_SmoothPath;

public:
	PathPlanner(const PathFinding::Config& pfConfig);

	void PlanPath(const Map& map, const Pose& startPose, Vector2 destPos, float destRadius);
	void ResetPath();

	bool PathValid() const { return !m_SmoothPath.empty(); }
	const std::vector<Vector2>& GetPath() const { return m_SmoothPath; }
};