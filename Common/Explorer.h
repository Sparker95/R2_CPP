#pragma once

#include "Map.h"

class PathCoordinator;
struct Pose;

// Explorer is ExplorationManager. It selects new map positions for exploration and commands the PathCoordinator.

struct ExplorerConfig
{
	float m_DistMapThreshold_m = 0.2f; // _m stands for "meters"
	float m_MoveRadius_m = 0.2f;
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