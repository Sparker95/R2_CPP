#include "ControlPanelServer.h"
#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Common.h"
#include "../Common/Utils.h"
#include "../Common/Scan.h"
#include "../Common/Map.h"
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

static char g_SendBuffer[ControlPanelProtocol::MAX_PAYLOAD_SIZE];

using BufferWriter = ControlPanelProtocol::BufferWriter;
using BufferReader = ControlPanelProtocol::BufferReader;

void ControlPanelServer::OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
{
	static int __test = 0; // WIP don't commit

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
		case (MessageType::MSG_ENABLE_STREAM_SENSOR_DATA):
		{
			if (msgPayloadSize != 1)
				return;

			m_bStreamSensorData = reader.Read<uint8_t>() == 1;
			break;
		}
		case (MessageType::MSG_REQUEST_MAP):
		{
			// Send back a map
			uint xSize = 256; uint ySize = 256;
			Map m(xSize, ySize, 128, 128, 0.0625);
			for (uint y = 0; y < ySize; y++)
			{
				for (uint x = 0; x < xSize; x++)
				{
					m.SetDataAtPos(Map::TCellPos(x, y), static_cast<Map::TCellData>((8*(x + y + __test*13)) & 0xFF));
				}
			}
			__test++;

			// Serialize map
			BufferWriter writer(g_SendBuffer, sizeof(g_SendBuffer));

			ControlPanelProtocol::MapDataHeader mapHeader;
			mapHeader.m_Res_w = m.GetResolution_w();
			mapHeader.m_xSize = m.GetSize().x;
			mapHeader.m_ySize = m.GetSize().y;
			mapHeader.m_xCenter = m.GetCenter().x;
			mapHeader.m_yCenter = m.GetCenter().y;

			writer.Write(mapHeader);

			for (uint y = 0; y < m.GetSize().y; y++)
			{
				for (uint x = 0; x < m.GetSize().x; x++)
					writer.Write<Map::TCellData>(m.GetDataAtPos(Map::TCellPos(x, y)));
			}

			SendMessage(MessageType::MSG_MAP, writer.GetNumBytesWritten(), g_SendBuffer);

			break;
		}
		default:
			break;
	}
}

void ControlPanelServer::ProcessPipeMessage(PipeMsgBase* pMsg, bool& outTerminate)
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
		header.m_nSamples = static_cast<uint32_t>(pMsgScanData->m_ScanSamples.size());

		writer.Write(header);

		for (const auto& sample : pMsgScanData->m_ScanSamples)
			writer.Write(sample);

		SendMessage(ControlPanelProtocol::MessageType::MSG_SENSOR_DATA, writer.GetNumBytesWritten(), g_SendBuffer);
	}
}