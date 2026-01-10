#include "ControlPanelServer.h"
#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Common.h"
#include "../Common/Utils.h"
#include "../Common/Scan.h"
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
		case (MessageType::MSG_ENABLE_STREAM_SENSOR_DATA):
		{
			if (msgPayloadSize != 1)
				return;

			m_bStreamSensorData = reader.Read<uint8_t>() == 1;
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