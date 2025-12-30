#include "StringWordReader.h"
#include "CommonIntTypes.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

StringWordReader::StringWordReader(const char* pData, uint dataSize) :
	m_pData(pData),
	m_pDataEnd(pData + dataSize)
{
	for (uint i = 0; i < WORD_BUFFER_SIZE; i++)
		m_WordBuffer[i] = 0;
}

bool StringWordReader::ReadWord()
{
	for (uint i = 0; i < WORD_BUFFER_SIZE; i++)
		m_WordBuffer[i] = 0;

	// Skip leading separators
	while (true)
	{
		if (m_pData >= m_pDataEnd)
			return false;
		if (!IsSeparatorCharacter(*m_pData))
			break;
		else
			m_pData++;
	}

	// Read into the word buffer, until next separator or until end
	uint iDst = 0;
	while (true)
	{
		// Verify that word is not too long
		if (iDst >= MAX_WORD_SIZE - 1)
			return false;

		m_WordBuffer[iDst] = *m_pData;
		m_pData++;
		iDst++;

		if (m_pData >= m_pDataEnd)
			break;
		if (IsSeparatorCharacter(*m_pData))
			break;
	}

	return true;
}

template<>
bool StringWordReader::Read(int& outValue, bool reportError)
{
	if (!ReadWord())
	{
		if (reportError)
			std::cerr << "StringWordReader: Error: Failed to read word, expected int\n";
		outValue = 0;
		return false;
	}

	outValue = std::atoi(m_WordBuffer);
	return true;
}

template<>
bool StringWordReader::Read(uint& outValue, bool reportError)
{
	if (!ReadWord())
	{
		if (reportError)
			std::cerr << "StringWordReader: Error: Failed to read word, expected uint\n";
		outValue = 0;
		return false;
	}

	long long value = std::atoll(m_WordBuffer);
	outValue = static_cast<uint>(value);
	return true;
}

template<>
bool StringWordReader::Read(float& outValue, bool reportError)
{
	if (!ReadWord())
	{
		if (reportError)
			std::cerr << "StringWordReader: Error: Failed to read word, expected float\n";
		outValue = 0.0f;
		return false;
	}

	double value = std::atof(m_WordBuffer);
	outValue = static_cast<float>(value);
	return true;
}

template<>
bool StringWordReader::Read(const char*& outValue, bool reportError)
{
	if (!ReadWord())
	{
		if (reportError)
			std::cerr << "StringWordReader: Error: Failed to read word, expected string\n";
		outValue = nullptr;
		return false;
	}

	outValue = m_WordBuffer;
	return true;
}

bool StringWordReader::ReadCompareWord(const char* pStr, bool reportError /*= true*/)
{
	const char* pWord = nullptr;
	if (!Read<const char*>(pWord, false))
	{
		if (reportError)
			std::cerr << "StringWordReader: Error: Failed to read word, expected word: " << pStr << "\n";
		return false;
	}

	bool result = strcmp(pWord, pStr) == 0;
	if (!result && reportError)
		std::cerr << "StringWordReader: Error: Unexpected word: " << m_WordBuffer << ", Expected: " << pStr << "\n";
	return result;
}

bool StringWordReader::End() const
{
	// Return true if the trailing characters until the end are separators
	const char* pData = m_pData;
	while (true)
	{
		// Reached end -> it's end
		if (pData >= m_pDataEnd)
			return true;

		
		if (IsSeparatorCharacter(*pData))
		{
			// It's separator, keep moving
			pData++;
		}
		else
		{
			// Reached non-separator value -> it's not end
			return false;
		}
	}

	return false;
}

bool StringWordReader::IsSeparatorCharacter(char c)
{
	return c == ' ' || c == 10 || c == 13 || c == 0;
}
