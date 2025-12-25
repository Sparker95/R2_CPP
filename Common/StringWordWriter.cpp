#include "StringWordWriter.h"
#include "IntTypes.h"

#include <cstdio>

StringWordWriter::StringWordWriter(std::string& outString)
{
	m_pOutString = &outString;
}

template<>
void StringWordWriter::Write(int value)
{
	char wordBuffer[WORD_BUFFER_SIZE];
	wordBuffer[WORD_BUFFER_SIZE - 1] = 0;
	snprintf(wordBuffer, WORD_BUFFER_SIZE, "%i ", value);
	m_pOutString->append(wordBuffer);
}

template<>
void StringWordWriter::Write(uint value)
{
	char wordBuffer[WORD_BUFFER_SIZE];
	wordBuffer[WORD_BUFFER_SIZE - 1] = 0;
	snprintf(wordBuffer, WORD_BUFFER_SIZE, "%u ", value);
	m_pOutString->append(wordBuffer);
}

void StringWordWriter::Write(float value, const char* pFormat)
{
	char wordBuffer[WORD_BUFFER_SIZE];
	wordBuffer[WORD_BUFFER_SIZE - 1] = 0;
	snprintf(wordBuffer, WORD_BUFFER_SIZE, pFormat, value);
	m_pOutString->append(wordBuffer);
	m_pOutString->append(" ");
}

template<>
void StringWordWriter::Write(const char* value)
{
	m_pOutString->append(value);
	m_pOutString->append(" ");
}

void StringWordWriter::WriteNewLine()
{
	m_pOutString->append("\n");
}
