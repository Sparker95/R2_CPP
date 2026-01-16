#pragma once

#include "ControlSensorFrame.h"
#include "StringWordReader.h"

#include <vector>

namespace ControlSensorFrameUtils
{
	bool ReadFrameFromString(StringWordReader& reader, ControlSensorFrame& outBlock);
	bool ReadFramesFromFile(const char* pFilePath, std::vector<ControlSensorFrame>& outBlocks);
}