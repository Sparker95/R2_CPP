#include "Config.h"
#include "../Common/Common.h"

#include <fstream>
#include <string.h>
#include <cstdlib>

Config::Config()
{

}

Config::~Config()
{
	if (m_pConfigData)
		delete[] m_pConfigData;
}

const char* Config::FindValue(const char* pKeyName, bool printError /*= true*/) const
{
	for (auto& kv : m_KeyValueTable)
	{
		if (strcmp(kv.m_pKey, pKeyName) == 0)
			return kv.m_pValue;
	}

	if (printError)
		fprintf(stderr, "Config: Error: key not found: %s\n", pKeyName);

	return nullptr;
}

bool Config::ReadConfig(const char* configPath)
{
	printf("Config: Reading file: %s\n", configPath);

	std::ifstream cfgFile(configPath, std::ios::in);

	if (!cfgFile.is_open())
	{
		fprintf(stderr, "Config: Error opening config file: %s\n", configPath);
		return false;
	}

	cfgFile.seekg(0, std::ios_base::end);
	uint cfgFileSize = static_cast<uint>(cfgFile.tellg());
	cfgFile.seekg(0, std::ios_base::beg);

	if (m_pConfigData)
		delete[] m_pConfigData;

	m_pConfigData = new char[cfgFileSize+1]; // Extra byte for null character
	cfgFile.read(m_pConfigData, cfgFileSize);
	m_pConfigData[cfgFileSize] = 0;

	// Split data into key-value pairs
	char* pLineStart = m_pConfigData;
	while (m_KeyValueTable.size() < 64) // Sanity check
	{
		char* pSpace = strchr(pLineStart, ' ');
		char* pNewLine = strchr(pLineStart, '\n');
		if (pSpace == nullptr || (pSpace > pNewLine && pNewLine != nullptr))
		{
			fprintf(stderr, "Config: File data error\n");
			return false;
		}

		m_KeyValueTable.push_back({pLineStart, pSpace+1});

		// Split strings with 0, to help future parsing
		*pSpace = 0;
		if (pNewLine)
			*pNewLine = 0;
		
		// Reached the end
		if (pNewLine == nullptr)
			break;

		pLineStart = pNewLine + 1;
	}

	// Print the found data
	printf("Config: Config file data: %i entries:\n", m_KeyValueTable.size());
	for (uint i = 0; i < m_KeyValueTable.size(); i++)
	{
		const auto& kv = m_KeyValueTable[i];
		printf("  %i: %s : %s\n", i, kv.m_pKey, kv.m_pValue);
	}

	return true;
}

bool Config::Get(const char* pKeyName, float& outValue, bool reportError /*= true*/) const
{
	const char* pValueStr = FindValue(pKeyName, reportError);

	if (!pValueStr)
		return false;

	outValue = static_cast<float>(std::atof(pValueStr));

	return true;
}

bool Config::Get(const char* pKeyName, uint& outValue, bool reportError /*= true*/) const
{
	const char* pValueStr = FindValue(pKeyName, reportError);

	if (!pValueStr)
		return false;

	outValue = static_cast<uint>(std::atol(pValueStr));

	return true;
}