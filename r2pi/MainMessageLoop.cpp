#include "MainMessageLoop.h"

#include "SensorInterface.h"
#include "ChassisInterface.h"
#include "ControlPanelServer.h"

#include <stdio.h>

//------------------------------------------------------------------------------------------------------------------------
// Constructor, destructor

MainMessageLoop::MainMessageLoop(SensorInterface* pSensorInterface, ChassisInterface* pChassisInterface)
{
	m_ScanSamples.reserve(Sensor::SCAN_N_SAMPLES_MAX);
	m_pSensorInterface = pSensorInterface;
	m_pChassisInterface = pChassisInterface;

	m_pSensorInterface->BindOnReceivedScan(
		[this](const Sensor::ScanSample* pSamples, uint nValidSamples, uint nRawSamples)
		{ OnReceivedScan_mt(pSamples, nValidSamples, nRawSamples); } );

	m_TimeStart = std::chrono::high_resolution_clock::now();
}

void MainMessageLoop::Init(ControlPanelServer* pCtrlPanelServer)
{
	m_pCtrlPanelServer = pCtrlPanelServer;
}

MainMessageLoop::~MainMessageLoop()
{
}

//------------------------------------------------------------------------------------------------------------------------
// Message loop

void MainMessageLoop::Execute()
{
	while (1)
	{
		std::unique_lock queueLock(m_QueueMutex);

		if (m_MessageQueue.empty())
		{
			m_QueueCondVariable.wait_for(queueLock, std::chrono::milliseconds(250));
			// now queueLock is locked
		}

		// Following cases are possible:
		// - It was not empty
		// - It was empty and got pushed
		// - It was empty and cv timed out
		// Either way check termination signal
		if (m_SignalValue.load() != 0)
		{
			printf("MainMessageLoop: Received signal from OS: %i\n", m_SignalValue.load());
			break;
		}

		if (m_MessageQueue.empty())
			continue;

		std::unique_ptr<MessageBase> pMsg = std::move(m_MessageQueue.front());
		m_MessageQueue.pop();
		queueLock.unlock(); // Unlock queue sooner, don't hold it while processing messaage

		// Process message
		{
			// The mutex is locked whenever we are processing message,
			// So that other threads could read safely our data
			std::unique_lock stateLock(m_StateMutex);
			ProcessMessage(*pMsg.get());
		}
	}

	printf("MainMessageLoop: returned from Execute()\n");
}

void MainMessageLoop::PostMessage(std::unique_ptr<MessageBase>&& pMsg)
{
	std::unique_lock queueLock(m_QueueMutex);
	m_MessageQueue.push(std::move(pMsg));
	m_QueueCondVariable.notify_one();
}

void MainMessageLoop::HandleOsSignal(int signalValue)
{
	m_SignalValue = signalValue;
}

//------------------------------------------------------------------------------------------------------------------------
// Message processing

void MainMessageLoop::ProcessMessage(MessageBase& msgBase)
{
	if (MessageNewSensorScanData* pMsg = dynamic_cast<MessageNewSensorScanData*>(&msgBase))
	{
		// WIP later sensor itself will provide time stamp
		auto timeNow = std::chrono::high_resolution_clock::now();
		uint timestamp_ms = static_cast<uint>(std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - m_TimeStart).count());

		m_ScanSamples.swap(pMsg->m_Scan);
		m_ScanCounter++;
		//printf("MainMessageLoop: Received new scan: %i raw samples\n", pMsg->m_RawSampleCount);

		// Read chassis data
		using TMoveState = ChassisInterface::MoveState;
		TMoveState moveState = {};
		ChassisInterface::TOdometerCounter odoCounter = 0;
		float steer = 0;

		{
			std::unique_lock chassisLock(m_pChassisInterface->GetMutex());
			m_pChassisInterface->SendCommand_GetOdometer(odoCounter, moveState);
			m_pChassisInterface->SendCommand_GetSteer(steer);

			if (m_pCtrlPanelServer->GetConnected() && m_pCtrlPanelServer->GetStreamSensorData())
			{
				m_pCtrlPanelServer->SendSensorData_mt(timestamp_ms, pMsg->m_Scan, odoCounter, moveState, steer);
			}
		}

		// Print debug text
		const char* moveStateText = "";
		switch (moveState)
		{
		case TMoveState::BACKWARD: moveStateText = "BWD"; break;
		case TMoveState::FORWARD: moveStateText = "FWD"; break;
		case TMoveState::STOP: moveStateText = "STOP"; break;
		}
		printf("%i, Scan Size: %i, Odometer: %i, MoveState: %s, Steer: %.2f\n",
			timestamp_ms, pMsg->m_Scan.size(), static_cast<int>(odoCounter), moveStateText, steer);

		return;
	}

	fprintf(stderr, "MainMessageLoop: Received unknown message type!\n");
}

//------------------------------------------------------------------------------------------------------------------------
// Connection with other objects

void MainMessageLoop::OnReceivedScan_mt(const Sensor::ScanSample* pSamples, uint nValidSamples, uint nRawSamples)
{
	MessageNewSensorScanData* pMsg = new MessageNewSensorScanData();

	pMsg->m_RawSampleCount = nRawSamples;
	pMsg->m_Scan.resize(nValidSamples);
	for (uint i = 0; i < nValidSamples; i++)
		pMsg->m_Scan[i] = pSamples[i];

	PostMessage(std::unique_ptr<MessageBase>(pMsg));
}

//------------------------------------------------------------------------------------------------------------------------
// Getters 

void MainMessageLoop::GetScanSamples_mt(std::vector<Sensor::ScanSample>& outSamples, uint& outScanCounter)
{
	std::unique_lock lock(m_StateMutex);
	outSamples = m_ScanSamples;
	outScanCounter = m_ScanCounter;
}