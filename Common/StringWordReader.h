#pragma once

#include "IntTypes.h"

class StringWordReader
{
private:
	constexpr static uint MAX_WORD_SIZE = 63;
	constexpr static uint WORD_BUFFER_SIZE = MAX_WORD_SIZE + 1; // Leading 0 terminator

	const char* m_pData = nullptr;
	const char* m_pDataEnd = nullptr;
	char m_WordBuffer[WORD_BUFFER_SIZE];

	bool ReadWord();
	static bool IsSeparatorCharacter(char c);

public:
	StringWordReader(const char* pData, uint dataSize);

	// Reading values
	template<class T>
	bool Read(T& outValue, bool reportError = true) = delete;

	template<>
	bool Read(uint& outValue, bool reportError);

	template<>
	bool Read(int& outValue, bool reportError);

	template<>
	bool Read(float& outValue, bool reportError);

	// Reads C-style string
	template<>
	bool Read(const char*& outValue, bool reportError);

	// Reads word and returns true if it matches the provided string
	bool ReadCompareWord(const char* pStr, bool reportError = true);

	// Returns true when there are no more words to read
	bool End() const;
};