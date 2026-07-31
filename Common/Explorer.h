#pragma once

#include "Map.h"

#include <vector>

class PathCoordinator;
class Localizer;
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
	enum class State
	{
		IDLE,
		ACTIVE,
		ERROR
	};

private:
	Localizer* m_pLocalizer = nullptr;
	PathCoordinator* m_pPathCoordinator = nullptr;
	const ExplorerConfig m_Config;
	State m_State = State::IDLE;
	std::vector<Vector2> m_AltExplorePoints;

	bool FindExplorePosAndMove();

public:
	Explorer(Localizer& localizer, PathCoordinator& pathCoordinator, const ExplorerConfig& config);

	void StartExploration();
	void Update();
};