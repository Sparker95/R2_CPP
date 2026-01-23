#include "FileUtils.h"

#include <string>
#include <stdio.h>
#include <fstream>

using uint = unsigned int;

bool FileUtils::FindSafeFileName(const char* fileNameNoExt, const char* fileExt, std::string& outSafeFileName)
{
	char fileNameBuffer[1024];

	// Find suitable file name.
	// First try trivial case FileName.txt, then FileName (1).txt and so on.

	snprintf(fileNameBuffer, sizeof(fileNameBuffer), "%s%s", fileNameNoExt, fileExt);

	if (!FileExists(fileNameBuffer))
	{
		outSafeFileName = fileNameBuffer;
		return true;
	}
	else
	{
		for (uint i = 1; i < 1024; i++)
		{
			snprintf(fileNameBuffer, sizeof(fileNameBuffer), "%s (%u)%s", fileNameNoExt, i, fileExt);
			if (!FileExists(fileNameBuffer))
			{
				outSafeFileName = fileNameBuffer;
				return true;
			}
		}
	}

	return false;
}

bool FileUtils::FileExists(const char *pFileName)
{
	std::ifstream f(pFileName);
	return f.good();
}


