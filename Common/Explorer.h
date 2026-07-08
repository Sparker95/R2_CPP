#pragma once

#include "Map.h"

class PathCoordinator;
struct Pose;

// Explorer is ExplorationManager. It selects new map positions for exploration and commands the PathCoordinator.

struct ExplorerConfig
{
	float m_DistMapThreshold = 0.2f; // _m stands for "meters"
	float m_MoveRadius = 0.2f;
	float m_MinDistToCell = 0.35f; // Closer cells are ignored, movement to them is problematic
};

class Explorer
{
private:
	PathCoordinator* m_pPathCoordinator = nullptr;
	const ExplorerConfig m_Config;

public:
	Explorer(PathCoordinator& pathCoordinator, const ExplorerConfig& config);

	void ExploreNextPosition(const OccupancyMap& map, const Pose& pose);
};