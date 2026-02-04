#pragma once

#include "Localizer.h"

class Config;

namespace LocalizerUtils
{
	bool WriteLocalizerHistoryToFile(const Localizer& localizer, const char* pFilePath, uint nEntries);

	bool ReadConfig(const Config& config, Localizer::Config& outConfig);
}