#include "Config.h"

#include <fstream>
#include <string.h>
#include <cstdlib>

Config::Config()
{

}

Config::Config(const char* configPath)
{
	ReadConfig(configPath);
}

Config::~Config()
{
	if (m_pConfigData)
		delete[] m_pConfigData;
}

const char* Config::FindValue(const char* pSectionName, const char* pKeyName, bool printError /*= true*/) const
{
	for (auto& kv : m_KeyValueTable)
	{
		if (strcmp(kv.m_pSection, pSectionName) == 0 &&
			strcmp(kv.m_pKey, pKeyName) == 0)
			return kv.m_pValue;
	}

	if (printError)
		fprintf(stderr, "Config: Error: key not found: %s.%s\n", pSectionName, pKeyName);

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
	char* pSection = nullptr; // Current section
	while (m_KeyValueTable.size() < 64) // Sanity check
	{
		char* pSpace = strchr(pLineStart, ' ');
		char* pNewLine = strchr(pLineStart, '\n');

		if (*pLineStart == '#')          // Comment line -> ignore
		{ /* do nothing */ }
		else if (pNewLine == pLineStart) // Empty line -> ignore
		{ /* do nothing */ }
		else if (*pLineStart == '-')     // Section declaration
		{
			pSection = pLineStart + 1;
		}
		else                             // Probably key-value pair
		{
			if (!pSpace || (pSpace && pNewLine && (pSpace > pNewLine))) // Line without space
			{
				fprintf(stderr, "Config: File data error: key-value line without space\n");
				return false;
			}

			if (pSection)
			{
				KeyValuePair pair;
				pair.m_pSection = pSection;
				pair.m_pKey = pLineStart;
				pair.m_pValue = pSpace + 1;

				if (pSpace)
					*pSpace = 0;

				m_KeyValueTable.push_back(pair);
			}
			else
			{
				fprintf(stderr, "Config: Section-less key-value pairs are not supported!\n");
				return false;
			}
		}	

		// Split strings with 0, to help future processing (strcmp and other functions)
		if (pNewLine)
			*pNewLine = 0;
		
		// Reached the end
		if (pNewLine == nullptr)
			break;

		pLineStart = pNewLine + 1;
		if (pLineStart >= m_pConfigData + cfgFileSize)
			break;
	}

	// Print the found data
	printf("Config: Config file data: %i entries:\n", static_cast<int>(m_KeyValueTable.size()));
	for (uint i = 0; i < m_KeyValueTable.size(); i++)
	{
		const auto& kv = m_KeyValueTable[i];

		if (!kv.m_pSection)
			printf("  %i: %s : %s\n", i, kv.m_pKey, kv.m_pValue);
		else
			printf("  %i: %s.%s : %s\n", i, kv.m_pSection, kv.m_pKey, kv.m_pValue);
	}

	return true;
}

void Config::SetSection(const char* pSectionName) const
{
	// This is fine, this way we can use SetSection on const objects.
	const_cast<Config*>(this)->m_pCurrentSection = pSectionName;
}

bool Config::Get(const char* pKeyName, float& outValue, bool reportError /*= true*/) const
{
	const char* pValueStr = FindValue(m_pCurrentSection, pKeyName, reportError);

	if (!pValueStr)
		return false;

	outValue = static_cast<float>(std::atof(pValueStr));

	return true;
}

bool Config::Get(const char* pKeyName, uint& outValue, bool reportError /*= true*/) const
{
	const char* pValueStr = FindValue(m_pCurrentSection, pKeyName, reportError);

	if (!pValueStr)
		return false;

	outValue = static_cast<uint>(std::atol(pValueStr));

	return true;
}

bool Config::Get(const char* pKeyName, bool& outValue, bool reportError /*= true*/) const
{
	const char* pValueStr = FindValue(m_pCurrentSection, pKeyName, reportError);

	if (!pValueStr)
		return false;

	outValue = *pValueStr == '1';

	return true;
}

bool Config::Get(const char* pKeyName, std::string& outValue, bool reportError /*= true*/) const
{
	const char* pValueStr = FindValue(m_pCurrentSection, pKeyName, reportError);

	if (!pValueStr)
		return false;

	outValue = pValueStr;

	return true;
}