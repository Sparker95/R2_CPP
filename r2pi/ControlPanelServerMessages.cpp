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
#include <errno.h>

static char g_SendBuffer[16 * 1024];

using BufferWriter = ControlPanelProtocol::BufferWriter;
using BufferReader = ControlPanelProtocol::BufferReader;

void ControlPanelServer::OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
{
	using MessageType = ControlPanelProtocol::MessageType;
	BufferReader reader(msgPayload, msgPayloadSize);
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
		case (MessageType::MSG_ENABLE_STREAM_SENSOR_DATA):
		{
			if (msgPayloadSize != 1)
				return;

			m_bStreamSensorData = reader.Read<uint8_t>() == 1;
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
		default:
			break;
	}
}

// Called on pipe new message, also handles termination of thread
bool ControlPanelServer::ProcessPipeMessage()
{
	PipeMsgBase* pMsg = nullptr;
	ssize_t nBytesRead = read(m_PipeFd[0], &pMsg, sizeof(PipeMsgBase*));

	if (nBytesRead != sizeof(PipeMsgBase*))
	{
		fprintf(stderr, "ControlPanelServer: pipe read error: %i\n", errno);
		return false;
	}

	bool terminateThread = false;
	OnPipeMessage(pMsg, terminateThread);

	// Remember to delete message!
	delete pMsg; // Virtual destructor!

	return !terminateThread;
}

void ControlPanelServer::OnPipeMessage(PipeMsgBase* pMsg, bool& outTerminate)
{
	BufferWriter writer(g_SendBuffer, sizeof(g_SendBuffer));
	
	if (dynamic_cast<PipeMsgTerminate*>(pMsg))
	{
		outTerminate = true;
		return;
	}
	else if (PipeMsgSensorData* pMsgScanData = dynamic_cast<PipeMsgSensorData*>(pMsg))
	{
		if (!GetConnected() || !GetStreamSensorData())
			return;

		ControlPanelProtocol::SensorDataHeader header;

		header.m_Timestamp_ms = pMsgScanData->m_Timestamp_ms;
		header.m_Odometer = pMsgScanData->m_Odometer;
		header.m_MoveState = pMsgScanData->m_MoveState;
		header.m_Steer = pMsgScanData->m_Steer;
		header.m_nSamples = static_cast<uint32_t>(pMsgScanData->m_Samples.size());

		writer.Write(header);

		for (const auto& sample : pMsgScanData->m_Samples)
			writer.Write(sample);

		SendMessage(ControlPanelProtocol::MessageType::MSG_SENSOR_DATA, writer.GetNumBytesWritten(), g_SendBuffer);
	}
}