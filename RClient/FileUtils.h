#ifndef FILEUTILS_H
#define FILEUTILS_H

#include <string>

namespace FileUtils
{
	bool FileExists(const char* pFileName);

	// If file does not exist, returns fileNameNoExt + fileExt, ex.: "MyFile.txt"
	// Otherwise tries to find file with name fileNameNoExt + (N) + fileExt, ex.: "MyFile (3).txt"
	bool FindSafeFileName(const char* fileNameNoExt, const char* fileExt, std::string& outSafeFileName);
}


#endif // FILEUTILS_H
