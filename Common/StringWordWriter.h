#pragma once

#include "IntTypes.h"

#include <string>

class StringWordWriter
{
	constexpr static uint MAX_WORD_SIZE = 63;
	constexpr static uint WORD_BUFFER_SIZE = MAX_WORD_SIZE + 1; // Leading 0 terminator

private:
	std::string* m_pOutString = nullptr;

public:
	StringWordWriter(std::string& outString);

	template<class T>
	void Write(T value) = delete;

	template<>
	void Write(int value);

	template<>
	void Write(uint value);

	void Write(float value, const char* pFormat = "%f");

	template<>
	void Write(const char* value);

	void WriteNewLine();
};