#pragma once

#include "SensorStream.h"
#include "StringWordReader.h"

#include <vector>

namespace SensorStreamUtils
{
	bool ReadBlockFromString(StringWordReader& reader, SensorStreamBlock& outBlock);
	bool ReadBlocksFromFile(const char* pFilePath, std::vector<SensorStreamBlock>& outBlocks);
}