#pragma once

#include "CommonIntTypes.h"

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

	void Write(float value, const char* pFormat = "%f");

	void WriteNewLine();
};

template<>
void StringWordWriter::Write(int value);

template<>
void StringWordWriter::Write(uint value);

template<>
void StringWordWriter::Write(const char* value);