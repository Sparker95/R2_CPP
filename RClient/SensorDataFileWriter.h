#ifndef SENSORDATAFILEWRITER_H
#define SENSORDATAFILEWRITER_H

#include <fstream>
#include <string>

#include <QList>

#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"


class SensorDataFileWriter
{
    using uint = unsigned int;

public:
    void Start(const char* pFileNameNoExt);
    void Stop();
    void WriteData(ControlPanelProtocol::SensorDataHeader& header, QList<Sensor::ScanSample>& scan);
    bool GetIsOpen() const;
    void GetStats(uint& outFileSize, uint& outBlockCount) const;
    const std::string& GetFileName() const;

private:
    std::ofstream m_File;
    std::string m_FileName;
    std::string m_WriteBuffer;
    uint m_StatsBlockCounter = 0;
    uint m_StatsByteCounter = 0;

    static void AppendFormat(std::string& appendTo, const char* format, ...);
    static bool FileExists(const char* pFileName);
};

#endif // SENSORDATAFILEWRITER_H
