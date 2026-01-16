#pragma once

#include "CommonIntTypes.h"

#include <vector>

class Config
{
public:
	Config();
	~Config();
	bool ReadConfig(const char* configPath);
	const char* FindValue(const char* pKeyName, bool printError = true) const;

	bool Get(const char* pKeyName, float& outValue, bool reportError = true) const;
	bool Get(const char* pKeyName, uint& outValue, bool reportError = true) const;

private:
	struct KeyValuePair
	{
		const char* m_pKey = nullptr;
		const char* m_pValue = nullptr;
	};
	char* m_pConfigData = nullptr;
	std::vector<KeyValuePair> m_KeyValueTable;
};