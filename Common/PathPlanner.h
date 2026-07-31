#pragma once

#include "Pose.h"
#include "Vector2.h"
#include "Map.h"
#include "PathFinding.h"
#include "PathFollower.h"

struct PathFollowerConfig;

// Plans path and stores the result.
// WIP now separate PathPlanner object is not fully needed as it can be made stateless.
// Later it might serve path planning requests asynchronously.
class PathPlanner
{
	using uint = unsigned int;

private:
	const PathFinding::Config m_PFConfig;
	const PathFollowerConfig m_FollowerConfig;

	// Planning results:
	std::vector<PathFollower::PathPoint> m_Path;
	Vector2 m_PathDestPos;

	// Performs post processing of path such as smoothing and insertion of additional points.
	// Converts path finding result into path which can be passed to PathFollower.
	void ProcessPath(const std::vector<PathFinding::Node>& pfResultNodes, std::vector<PathFollower::PathPoint>& outPath) const;

public:
	PathPlanner(const PathFinding::Config& pfConfig, const PathFollowerConfig& followerConfig);

	void PlanPath(const OccupancyMap& map, const Pose& startPose, Vector2 destPos, float destRadius);
	void PlanUnobstructPath(const OccupancyMap& map, const Pose& startPose);
	void ResetPath();

	// Checks that path points are not obstructed, starting from currentPathPointId. Returns true if path is clear.
	bool CheckPathObstruction(const OccupancyMap& map, const std::vector<PathFollower::PathPoint>& path, float maxCheckDist,
		uint currentPathPointId, uint* pOutObstrPointId);

	// Checks if given pose is obstructed. Returns true if it's clear.
	// Pose is checked same way as during regular path finding, with direction quantization.
	bool CheckPoseObstruction(const OccupancyMap& map, const Pose& pose) const;

	bool PathValid() const { return !m_Path.empty(); }
	const std::vector<PathFollower::PathPoint>& GetPath() const { return m_Path; }
	const Vector2& GetPathDestPos() const { return m_PathDestPos; }
};