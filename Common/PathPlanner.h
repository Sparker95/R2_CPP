#pragma once

#include "Pose.h"
#include "Vector2.h"
#include "Map.h"
#include "PathFinding.h"
#include "PathFollower.h"

struct PathFollowerConfig;

// Plans path and stores the result in itself.
class PathPlanner
{
	using uint = unsigned int;

private:
	const PathFinding::Config m_PFConfig;
	const PathFollowerConfig m_FollowerConfig;
	std::vector<PathFollower::PathPoint> m_Path;

	// Performs post processing of path such as smoothing and insertion of additional points.
	// Converts path finding result into path which can be passed to PathFollower.
	void ProcessPath(const std::vector<PathFinding::Node>& pfResultNodes, std::vector<PathFollower::PathPoint>& outPath) const;

public:
	PathPlanner(const PathFinding::Config& pfConfig, const PathFollowerConfig& followerConfig);

	void PlanPath(const OccupancyMap& map, const Pose& startPose, Vector2 destPos, float destRadius);
	void PlanUnobstructPath(const OccupancyMap& map, const Pose& startPose);
	void ResetPath();

	// Checks that path points are not obstructed, starting from currentPathPointId. Returns true if path is clear.
	bool CheckPathObstruction(const OccupancyMap& map, const std::vector<PathFollower::PathPoint>& path, uint currentPathPointId,
		uint* pOutObstrPointId);

	// Checks if given pose is obstructed. Returns true if it's clear.
	// Pose is checked same way as during regular path finding, with direction quantization.
	bool CheckPoseObstruction(const OccupancyMap& map, const Pose& pose) const;

	bool PathValid() const { return !m_Path.empty(); }
	const std::vector<PathFollower::PathPoint>& GetPath() const { return m_Path; }
};