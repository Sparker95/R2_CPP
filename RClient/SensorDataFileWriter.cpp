#include "SensorDataFileWriter.h"

#include <string>
#include <stdio.h>

using uint = unsigned int;

void SensorDataFileWriter::Start(const char* pFileNameNoExt)
{
    if (GetIsOpen())
        return;

    if (strlen(pFileNameNoExt) == 0)
        pFileNameNoExt = "SensorStream";

    const char fileExtension[] = ".txt";

    // Find suitable file name.
    // First try trivial case FileName.txt, then FileName (1).txt and so on.
    std::string fileName = pFileNameNoExt;
    fileName.append(fileExtension);

    if (FileExists(fileName.data()))
    {
        for (uint i = 1; i < 1024; i++)
        {
            fileName.clear();
            AppendFormat(fileName, "%s (%u)%s", pFileNameNoExt, i, fileExtension);
            if (!FileExists(fileName.data()))
                break;
        }
    }

    m_File.open(fileName, std::ios::out | std::ios::app);

    if (!GetIsOpen())
    {
        Stop();
        return;
    }

    m_FileName = fileName;
    m_StatsBlockCounter = 0;
    m_StatsByteCounter = 0;
}

void SensorDataFileWriter::Stop()
{
    if (!GetIsOpen())
        return;
    m_File.close();

    m_FileName.clear();
    m_StatsBlockCounter = 0;
    m_StatsByteCounter = 0;
}

bool SensorDataFileWriter::GetIsOpen() const
{
    return m_File.is_open();
}

const std::string& SensorDataFileWriter::GetFileName() const
{
    return m_FileName;
}

void SensorDataFileWriter::WriteData(ControlPanelProtocol::SensorDataHeader &header, QList<Sensor::ScanSample> &scan)
{
    if (!GetIsOpen())
        return;

    m_WriteBuffer.clear();
    m_WriteBuffer.reserve(128 + 24 * scan.size());

    m_WriteBuffer.append("SENSOR_DATA_HEADER ");
    AppendFormat(m_WriteBuffer, "TIME %u ", static_cast<unsigned int>(header.m_Timestamp_ms));
    AppendFormat(m_WriteBuffer, "ODO %u ", static_cast<unsigned int>(header.m_Odometer));

    const char* pMoveStateStr = nullptr;
    switch (header.m_MoveState)
    {
    case Chassis::MoveState::FORWARD:  pMoveStateStr = "FWD"; break;
    case Chassis::MoveState::BACKWARD: pMoveStateStr = "BWD"; break;
    case Chassis::MoveState::STOP:     pMoveStateStr = "STOP"; break;
    }
    AppendFormat(m_WriteBuffer, "MV_STATE %s ", pMoveStateStr);
    AppendFormat(m_WriteBuffer, "STEER %.3f\n", header.m_Steer);

    AppendFormat(m_WriteBuffer, "SCAN N_SAMPLES %u\n", static_cast<unsigned int>(scan.size()));

    unsigned int samplesInLineCounter = 0;
    for (unsigned int i = 0; i < scan.size(); i++)
    {
        AppendFormat(m_WriteBuffer, "SPL %.6f %.6f ", static_cast<double>(scan[i].m_Angle_rad),
                     static_cast<double>(scan[i].m_Dist_m));
        samplesInLineCounter++;
        if (samplesInLineCounter == 6 && scan.size() - i > 1)
        {
            m_WriteBuffer.append("\n");
            samplesInLineCounter = 0;
        }
    }
    m_WriteBuffer.append("\n");

    m_File.write(m_WriteBuffer.data(), m_WriteBuffer.size());
    m_StatsBlockCounter++;
    m_StatsByteCounter += m_WriteBuffer.size();
}

void SensorDataFileWriter::GetStats(uint& outFileSize, uint& outBlockCount) const
{
    outFileSize = m_StatsByteCounter;
    outBlockCount = m_StatsBlockCounter;
}

void SensorDataFileWriter::AppendFormat(std::string& appendTo, const char* format, ...)
{
    va_list args;
    char buff[1024];
    va_start(args, format);
    int strLength = vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);
    appendTo.append(buff, strLength);
}

bool SensorDataFileWriter::FileExists(const char* pFileName)
{
    std::ifstream f(pFileName);
    return f.good();
}


