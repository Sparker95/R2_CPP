#include "PathFollower.h"

#include "MathUtils.h"
#include "Utils.h"

#include <cmath>
#include <cstdio>

using uint = unsigned int;

PathFollower::PathFollower(const PathFollowerConfig& config)
{
	m_Config = config;
}

void PathFollower::FollowPath(const std::vector<PathPoint>& path)
{
	uint nPathPoints = (uint)path.size();
	uint nEndPoints = 0;
	uint nExtPoints = 0;
	for (const auto& p : path)
	{
		if (p.m_Type == PathPoint::Type::END)
			nEndPoints++;
		else if (p.m_Type == PathPoint::Type::EXTENSION)
			nExtPoints++;
	}

	printf("PathFollower::FollowPath: %u points, %u end points, %u extension points\n",
		nPathPoints, nEndPoints, nExtPoints);

	if (nPathPoints == 0 || nEndPoints == 0)
	{
		printf("PathFollower::FollowPath: Error: provided path is invalid\n");
		m_State = State::ERROR;
		return;
	}

	m_Path = path;

	m_CurrentPointId = 0;
	m_CurrentEndPointId = -1;
	for (uint i = 0; i < (uint)m_Path.size(); i++)
	{
		if (m_Path[i].m_Type == PathPoint::Type::END)
		{
			m_CurrentEndPointId = i;
			break;
		}
	}

	printf("PathFollower::FollowPath: Current point: %u, end point: %u\n", m_CurrentPointId, m_CurrentEndPointId);

	m_State = State::RUNNING;
}

void PathFollower::Reset()
{
	m_LookaheadPos = {};
	m_Path.clear();
	m_CurrentPointId = -1;
	m_CurrentEndPointId = -1;
	m_State = State::IDLE;

	m_CtrlCurvature = 0;
	m_CtrlSpeed = 0;
	m_CtrlBrake = true; // For safety
}

void PathFollower::Update(const Pose& pose)
{
	if (m_State != State::RUNNING)
		return;

	Vector2 vPoseDir(std::cos(pose.dir), std::sin(pose.dir));
	Vector2 currentPos(pose.x, pose.y);

	//------------------------------------------------------------------------------------
	// Step 1: Find the new current path point
	uint lastPtBehind = m_CurrentPointId;
	for (uint i = m_CurrentPointId; i < (uint)m_Path.size(); i++)
	{
		Vector2 vToPoint = m_Path[i].m_Pos - currentPos;
		float currentPtMoveDirSign = m_Path[m_CurrentPointId].m_OutMoveFwd ? 1.0f : -1.0f;
		if (currentPtMoveDirSign * Vector2::DotProduct(vToPoint, vPoseDir) < 0) // Point is behind current pose
		{
			lastPtBehind = i;
			if (i == m_CurrentEndPointId) // Don't seek beyond the current end point
				break;
		}
		else
			break;
	}

	if (lastPtBehind != m_CurrentPointId)
	{
		m_CurrentPointId = lastPtBehind;
		printf("PathFollower::Update: Reached point: %u\n", m_CurrentPointId);
	}

	const float compRadiusSq = m_Config.m_CompletionRadius * m_Config.m_CompletionRadius;
	if (m_CurrentPointId >= m_CurrentEndPointId ||
		Vector2::DistanceSq(currentPos, m_Path[m_CurrentEndPointId].m_Pos) < compRadiusSq)
	{
		printf("PathFollower::Update: Reached current end point: %u\n", m_CurrentEndPointId);
		
		// New current point - the extension point, should be next after end pt.
		// New end point - search starting from the new current point

		uint newCurrentPtId = -1;
		for (uint i = m_CurrentPointId; i < (uint)m_Path.size(); i++)
		{
			if (m_Path[i].m_Type == PathPoint::Type::EXTENSION)
			{
				newCurrentPtId = i;
				break;
			}
		}

		if (newCurrentPtId == (uint)-1)
		{
			printf("PathFollower::Update: Error: Failed to find next extension point, path is invalid\n");
			m_State = State::ERROR;
			return;
		}
		else if (newCurrentPtId == (uint)m_Path.size() - 1) // Last extension point
		{
			printf("PathFollower::Update: Reached end of path\n");
			m_State = State::END;
			return;
		}

		uint newEndPtId = -1;
		for (uint i = newCurrentPtId; i < (uint)m_Path.size(); i++)
		{
			if (m_Path[i].m_Type == PathPoint::Type::END)
			{
				newEndPtId = i;
				break;
			}
		}

		if (newEndPtId == (uint)-1)
		{
			printf("PathFollower::Update: Error: Failed to find next end point, path is invalid\n");
			m_State = State::ERROR;
			return;
		}

		m_CurrentPointId = newCurrentPtId;
		m_CurrentEndPointId = newEndPtId;
		printf("PathFollower:Update: Switched to current point: %u, end point: %u\n", m_CurrentPointId, m_CurrentEndPointId);
	}

	//------------------------------------------------------------------------------------
	// Step 2: Steering solution

	// Find lookahead position
	bool lookaheadPosFound = false;
	Vector2 lookaheadPos;
	for (uint i = m_CurrentPointId; i < (uint)m_Path.size(); i++)
	{
		const PathPoint& point = m_Path[i];
		const PathPoint& nextPoint = m_Path[i + 1];

		// Find first intersection which is ahead
		Vector2 intersections[2] = {};
		uint nIntersections = MathUtils::CircleXLine(currentPos, m_Config.m_LookaheadDistance,
			point.m_Pos, nextPoint.m_Pos, intersections);
		if (nIntersections != 0)
		{
			for (uint intersection = 0; intersection < nIntersections; intersection++)
			{
				Vector2& intPos = intersections[intersection];
				float currentPtMoveDirSign = m_Path[m_CurrentPointId].m_OutMoveFwd ? 1.0f : -1.0f;
				float dotProduct = Vector2::DotProduct(intPos - currentPos, vPoseDir);
				if (currentPtMoveDirSign * dotProduct >= 0.0f)
				{
					lookaheadPos = intPos;
					lookaheadPosFound = true;
					break;
				}
			}
		}

		if (lookaheadPosFound)
			break;
	}

	if (!lookaheadPosFound)
	{
		m_LookaheadPos = {};
		printf("PathFollower::Update: Error: Failed to find lookahead position\n");
		m_State = State::ERROR;
		return;
	}

	// Store lookahead pos for diagnostics
	m_LookaheadPos = lookaheadPos;

	// Calculate curvature
	// Implementation of the Pure Pursuit Path Tracking Algorithm, page 6
	Vector2 lookaheadPos_LS = Utils::PosWorldToLocal(pose, lookaheadPos);
	// Our system of coordinates is different from article, thus we use y coordinate
	float curvature = 2.0f * lookaheadPos_LS.y / (m_Config.m_LookaheadDistance * m_Config.m_LookaheadDistance);
	curvature = -curvature; // Positive - turn right;

	m_CtrlCurvature = curvature;

	// Calculate speed
	m_CtrlSpeed = m_Path[m_CurrentPointId].m_OutMoveFwd ? 1.0f : -1.0f;
}
