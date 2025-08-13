#include "ControlPanelServer.h"
#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Common.h"
#include "MainMessageLoop.h"

#include <vector>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>

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
			m_pMainMsgLoop->GetScanSamples_mt(samples);

			uint payloadSize = 4 + sizeof(Sensor::ScanSample) * static_cast<uint32_t>(samples.size()); // First number is num of samples

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
		default:
			break;
	}
}