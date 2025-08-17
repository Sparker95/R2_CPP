#pragma once

#include "Common.h"
#include "Scan.h"

namespace ICP
{
	Pose MatchScans(const Scan::TScanPolar& scan0Polar, const Scan::TScanPolar& scan1Polar, uint nIterMax, Pose initialT);
}