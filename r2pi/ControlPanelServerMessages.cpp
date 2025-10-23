#include "ControlPanelServer.h"
#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Common.h"
#include "../Common/Utils.h"
#include "../Common/Scan.h"
#include "../Common/ICP.h"
#include "MainMessageLoop.h"

#include <vector>
#include <thread>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static char g_SendBuffer[16 * 1024];

void ControlPanelServer::OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
{
	using MessageType = ControlPanelProtocol::MessageType;
	switch (static_cast<ControlPanelProtocol::MessageType>(msgType))
	{
		case (MessageType::MSG_PRINT):
		{
			// For safety, null terminate string
			if (msgPayloadSize < ControlPanelProtocol::MAX_PAYLOAD_SIZE - 1)
				msgPayload[msgPayloadSize] = 0;
			else
				msgPayload[ControlPanelProtocol::MAX_PAYLOAD_SIZE - 1] = 0;

			printf("Message: ");
			printf(msgPayload);
			printf("\n");

			break;
		}
		case (MessageType::MSG_TEST0):
		{
			// Send back some message
			ControlPanelProtocol::MessageHeader header;
			char text[] = "Hello from server!";
			header.Init(MessageType::MSG_PRINT, sizeof(text));
			send(m_ClientSockFd, static_cast<const void*>(&header), static_cast<size_t>(ControlPanelProtocol::MessageHeader::SIZE_PACKED), 0);
			send(m_ClientSockFd, static_cast<const void*>(text), static_cast<size_t>(header.m_PayloadSize), 0);
			break;
		}
		case (MessageType::MSG_RQ_SENSOR_SCAN_DATA):
		{
			// Send back sensor scan data
			std::vector<Sensor::ScanSample> samples;
			uint scanCounter;
			m_pMainMsgLoop->GetScanSamples_mt(samples, scanCounter);

			uint payloadSize = 4 + static_cast<uint>(sizeof(Sensor::ScanSample)) * static_cast<uint>(samples.size()); // First number is num of samples

			*reinterpret_cast<uint32_t*>(g_SendBuffer) = static_cast<uint32_t>(samples.size());
			Sensor::ScanSample* pOutSamples = reinterpret_cast<Sensor::ScanSample*>(&g_SendBuffer[4]);

			for (Sensor::ScanSample& s : samples)
			{
				*pOutSamples = s;
				pOutSamples++;
			}

			SendMessage(ControlPanelProtocol::MessageType::MSG_SENSOR_SCAN_DATA, payloadSize, g_SendBuffer);
			break;
		}
		case (MessageType::MSG_ICP_TEST):
		{
			if (msgPayloadSize != 1)
				return;

			if (*msgPayload == 0)
			{
				// Store scan for later comparison
				uint scanCounter;
				m_pMainMsgLoop->GetScanSamples_mt(m_IcpTestScan, scanCounter);
			}
			else
			{
				std::vector<Sensor::ScanSample> newScan;
				uint scanCounter;
				m_pMainMsgLoop->GetScanSamples_mt(newScan, scanCounter);

				Scan::TScanPolar oldScanPolar, newScanPolar;
				Utils::ScanSensorToMatrix(m_IcpTestScan.data(), static_cast<uint>(m_IcpTestScan.size()), oldScanPolar);
				Utils::ScanSensorToMatrix(newScan.data(), static_cast<uint>(newScan.size()), newScanPolar);

				Pose initialT;
				Pose resultT = ICP::MatchScans(newScanPolar, oldScanPolar, 16, initialT);

				char str[256];
				sprintf(str, "ICP result (dx, dy, ddir): %.3f %.3f %.3f", resultT.x, resultT.y, resultT.dir);

				// Send back message as text
				SendMessage(ControlPanelProtocol::MessageType::MSG_PRINT, static_cast<uint>(strlen(str)), str);
			}
		}
		case (MessageType::MSG_SAVE_SCANS):
		{
			if (msgPayloadSize != 1)
				return;

			char lineBuffer[256];

			unsigned int nScansToSave = static_cast<unsigned int>(*msgPayload);
			unsigned int scanCounter = m_pMainMsgLoop->GetScanCounter_mt();
			for (unsigned int scanId = 0; scanId < nScansToSave; scanId++)
			{
				// Wait till next scan
				while (true)
				{
					unsigned int newScanCounter = m_pMainMsgLoop->GetScanCounter_mt();
					if (newScanCounter != scanCounter)
						break;

					std::this_thread::sleep_for(std::chrono::milliseconds(50));
				}

				// We have waited till new scan
				std::vector<Sensor::ScanSample> samples;
				m_pMainMsgLoop->GetScanSamples_mt(samples, scanCounter);

				// Save to file
				char filePath[32];
				snprintf(filePath, 32, "Scan %i.txt", scanId);
				int fd = open(filePath, O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
				if (fd != -1)
				{
					snprintf(lineBuffer, 256, "%i\n", scanCounter);
					write(fd, lineBuffer, strlen(lineBuffer));

					for (const Sensor::ScanSample& sample : samples)
					{
						snprintf(lineBuffer, 256, "%.6f %.6f\n", sample.m_Angle_rad, sample.m_Dist_m);
						write(fd, lineBuffer, strlen(lineBuffer));
					}
				}
				close(fd);
			}

			// Finally report back that we have saved the scans
			snprintf(lineBuffer, 256, "Saved %i scans to file(s)", nScansToSave);
			SendMessage(ControlPanelProtocol::MessageType::MSG_PRINT, static_cast<uint>(strlen(lineBuffer)), lineBuffer);
		}
		default:
			break;
	}
}