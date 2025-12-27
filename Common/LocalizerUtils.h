#pragma once

#include "Localizer.h"

namespace LocalizerUtils
{
	bool WriteLocalizerHistoryToFile(const Localizer& localizer, const char* pFilePath, uint nEntries);
}