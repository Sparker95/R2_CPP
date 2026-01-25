#include "ControlPanelServer.h"
#include "../Common/ControlPanelProtocol.h"
#include "../Common/ControlInputsProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Common.h"
#include "../Common/Utils.h"
#include "../Common/Scan.h"
#include "../Common/Map.h"
#include "../Common/RobotCore.h"

#include "MainMessageLoop.h"
#include "SensorInterface.h"

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

using MessageType = ControlPanelProtocol::MessageType;

void ControlPanelServer::OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
{
	BufferReader reader(msgPayload, msgPayloadSize);
	switch (static_cast<ControlPanelProtocol::MessageType>(msgType))
	{
		case (MessageType::MSG_REQUEST_PROTOCOL_INFO):
		{
			// Send back protocol information
			BufferWriter writer(g_SendBuffer, sizeof(g_SendBuffer));
			writer.Write<uint8_t>(ControlPanelProtocol::PROTOCOL_VERSION);
			writer.Write<uint8_t>(ControlInputsProtocol::PROTOCOL_VERSION);
			SendMessage(MessageType::MSG_PROTOCOL_INFO, writer.GetNumBytesWritten(), g_SendBuffer);
			break;
		}
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

			m_bStreamSensorData = reader.Read<uint8_t>() != 0;
			break;
		}

		case (MessageType::MSG_REQUEST_OPTIONS):
		{
			SendOptions();
			break;
		}

		case (MessageType::MSG_ENABLE_SENSOR_MOTOR):
		{
			if (msgPayloadSize != 1)
				return;

			m_pSensorInterface->SetMotorState_mt(reader.Read<uint8_t>() != 0);

			break;
		}
				
		// Trivial messages for Main Message Loop are redirected to it, it decodes them itself
		case (MessageType::MSG_REQUEST_MAP):
		case (MessageType::MSG_REQUEST_LOC):
		case (MessageType::MSG_ENABLE_LOCALIZER):
		case (MessageType::MSG_RESET_LOCALIZER):
		{
			// Notitify main msg loop, it will send back map
			MainMessageLoop::MessageControlPanel* pMsgCtrl = new MainMessageLoop::MessageControlPanel();

			pMsgCtrl->m_Type = static_cast<ControlPanelProtocol::MessageType>(msgType);

			if (msgPayloadSize > 0)
			{
				if (msgPayloadSize > MainMessageLoop::MessageControlPanel::MAX_PAYLOAD_SIZE)
				{
					fprintf(stderr, "ControlPanelServer: Error: Message type %u has bigger payload (%u) than MainMessageLoop::MessageControlPanel supports(%u): %i, %i\n",
						static_cast<uint>(msgType), msgPayloadSize, MainMessageLoop::MessageControlPanel::MAX_PAYLOAD_SIZE);
					return;
				}

				std::memcpy(pMsgCtrl->m_Payload, msgPayload, msgPayloadSize);
			}

			m_pMainMsgLoop->PostMessage(std::unique_ptr<MainMessageLoop::MessageBase>(pMsgCtrl));

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
		// Send scan data back to client

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
	else if (PipeMsgMap* pMsgMap = dynamic_cast<PipeMsgMap*>(pMsg))
	{
		// Send map back to client

		if (!GetConnected())
			return;

		Map& m = pMsgMap->m_Map;

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
	}
	else if (PipeMsgLoc* pMsgLoc = dynamic_cast<PipeMsgLoc*>(pMsg))
	{
		if (!GetConnected())
			return;

		BufferWriter writer(g_SendBuffer, sizeof(g_SendBuffer));

		writer.Write<uint>(static_cast<uint>(pMsgLoc->m_Poses.size()));

		for (Pose& p : pMsgLoc->m_Poses)
			writer.Write(p);

		SendMessage(MessageType::MSG_LOC, writer.GetNumBytesWritten(), g_SendBuffer);
	}
	else if (dynamic_cast<PipeMsgSendOptions*>(pMsg))
	{
		SendOptions();
	}
}

void ControlPanelServer::FillOptionsValues(ControlPanelProtocol::OptionsHeader& optsHeader)
{
	using THeader = ControlPanelProtocol::OptionsHeader;

	if (m_bStreamSensorData)
		optsHeader.m_Options |= THeader::OPT_STREAM_SENSOR_DATA;

	if (m_pSensorInterface->GetMotorState_mt())
		optsHeader.m_Options |= THeader::OPT_SENSOR_MOTOR;

	if (m_pRobotCore->GetUpdateLocalizer())
		optsHeader.m_Options |= THeader::OPT_UPDATE_LOCALIZER;
}

void ControlPanelServer::SendOptions()
{
	// Fill options data
	ControlPanelProtocol::OptionsHeader optsHeader;
	FillOptionsValues(optsHeader);

	// Send it back
	BufferWriter writer(g_SendBuffer, sizeof(g_SendBuffer));
	writer.Write(optsHeader);
	SendMessage(MessageType::MSG_OPTIONS, writer.GetNumBytesWritten(), g_SendBuffer);
}