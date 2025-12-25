#include "SensorStreamUtils.h"
#include "IntTypes.h"

#include <string>
#include <fstream>

bool SensorStreamUtils::ReadBlockFromString(StringWordReader& reader, SensorStreamBlock& outBlock)
{
	// Read header
	if (!reader.ReadCompareWord("SENSOR_DATA_HEADER"))
		return false;

	if (!reader.ReadCompareWord("TIME"))
		return false;

	if (!reader.Read<uint>(outBlock.m_Timestamp_ms, true))
		return false;
	if (!reader.ReadCompareWord("ODO"))
		return false;

	uint _odometer;
	if (!reader.Read<uint>(_odometer, true))
		return false;

	outBlock.m_Odometer = static_cast<Chassis::TOdometerCounter>(_odometer);

	if (!reader.ReadCompareWord("MV_STATE"))
		return false;
	const char* pMvStateStr = nullptr;
	if (!reader.Read<const char*>(pMvStateStr, true))
		return false;
	if (!pMvStateStr)
		return false;

	if (std::strcmp(pMvStateStr, "FWD") == 0)
		outBlock.m_MoveState = Chassis::MoveState::FORWARD;
	else if (std::strcmp(pMvStateStr, "BWD") == 0)
		outBlock.m_MoveState = Chassis::MoveState::BACKWARD;
	else if (std::strcmp(pMvStateStr, "STOP") == 0)
		outBlock.m_MoveState = Chassis::MoveState::STOP;
	else
		return false;

	if (!reader.ReadCompareWord("STEER"))
		return false;
	if (!reader.Read<float>(outBlock.m_Steer, true))
		return false;

	// Read scan
	if (!reader.ReadCompareWord("SCAN"))
		return false;

	if (!reader.ReadCompareWord("N_SAMPLES"))
		return false;

	uint nSamples;
	if (!reader.Read<uint>(nSamples, true))
		return false;

	outBlock.m_ScanSamples.resize(nSamples);

	for (uint i = 0; i < nSamples; i++)
	{
		if (!reader.ReadCompareWord("SPL"))
			return false;

		Sensor::ScanSample& sample = outBlock.m_ScanSamples[i];

		if (!reader.Read<float>(sample.m_Angle_rad, true))
			return false;
		if (!reader.Read<float>(sample.m_Dist_m, true))
			return false;
	}

	return true;
}

bool SensorStreamUtils::ReadBlocksFromFile(const char* pFilePath, std::vector<SensorStreamBlock>& outBlocks)
{
	// Remember binary flag! Otherwise size measurement goes wrong, because of CRLF conversions.
	std::ifstream f(pFilePath, std::ios_base::in | std::ios_base::binary);
	if (!f.is_open())
		return false;

	// Read whole file into memory
	f.seekg(0, std::ios::end);
	uint fileSize = static_cast<uint>(f.tellg());
	f.seekg(0, std::ios::beg);

	std::string fileContents;
	fileContents.resize(fileSize, 0);
	f.read(fileContents.data(), fileSize);

	// Read blocks
	StringWordReader reader(fileContents.data(), fileSize);

	while (!reader.End())
	{
		SensorStreamBlock block;
		if (!ReadBlockFromString(reader, block))
			return false;

		outBlocks.push_back(std::move(block));
	}

	return true;
}