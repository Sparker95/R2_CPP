#pragma once

#include "CommonIntTypes.h"

#include <vector>
#include <string>

class Config
{
public:
	Config();
	Config(const char* configPath); // Calls ReadConfig() instantly
	~Config();
	bool ReadConfig(const char* configPath);
	const char* FindValue(const char* pKeyName, bool printError = true) const;

	bool Get(const char* pKeyName, float& outValue, bool reportError = true) const;
	bool Get(const char* pKeyName, uint& outValue, bool reportError = true) const;
	bool Get(const char* pKeyName, bool& outValue, bool reportError = true) const;
	bool Get(const char* pKeyName, std::string& outValue, bool reportError = true) const;

private:
	struct KeyValuePair
	{
		const char* m_pKey = nullptr;
		const char* m_pValue = nullptr;
	};
	char* m_pConfigData = nullptr;
	std::vector<KeyValuePair> m_KeyValueTable;
};