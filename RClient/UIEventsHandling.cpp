#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <cmath>

// Generic containers:
#include <QList>
#include <QPointF>

#include <QMessageBox>

#include "InputHandling.h"
#include "SensorScanWidget.h"
#include "FileUtils.h"
#include "MapWidget.h"

#include "../Common/ControlPanelProtocol.h"
#include "../Common/ControlInputsProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Pose.h"
#include "../Common/MapUtils.h"
#include "../Common/ProbUtils.h"
#include "../common/Localizer.h"

void MainWindow::SlotOnConnectButtonClicked()
{
    if (m_ControlInputsSocket.state() != QAbstractSocket::SocketState::BoundState)
    {
        bool controlsSocketBindRes = m_ControlInputsSocket.bind();
        if (!controlsSocketBindRes)
        {
            ui->ConnStateLabel->setText("UDP error");
            return;
        }
    }

    QString addressStr = ui->ConnAddress->text();
    m_ControlPanelSocket.connectToHost(addressStr, ControlPanelProtocol::CONTROL_PANEL_PORT);
    ui->ConnStateLabel->setText("Connecting ...");
}

void MainWindow::SlotOnEnableSensorStreamButton(bool checked)
{
	if (IsConnected())
    {
		char payload = checked ? 1 : 0;
        SendMessage(ControlPanelProtocol::MessageType::MSG_ENABLE_STREAM_SENSOR_DATA, 1, &payload);
    }
}

void MainWindow::SlotOnEnableSensorMotorButton(bool checked)
{
	if (IsConnected())
    {
		char payload = checked ? 1 : 0;
		SendMessage(ControlPanelProtocol::MessageType::MSG_ENABLE_SENSOR_MOTOR, 1, &payload);
    }
}

void MainWindow::SlotOnSaveSensorStreamToFileCheck(bool checked)
{
    // Open or close file and update UI
    if (checked)
    {
        QString fileNameEdit = ui->StreamFileNameEdit->text();
        std::string fileNameEditStd = fileNameEdit.toStdString();
        m_SensorFileWriter.Start(fileNameEditStd.data());
    }
    else
        m_SensorFileWriter.Stop();
    UpdateSensorFileStatsLabel();
}

void MainWindow::SlotOnSensorScaleValueChanged0(int value)
{
    m_pSensorScanWidget0->SetScale(static_cast<qreal>(value));
}

void MainWindow:: SlotOnSensorScaleValueChanged1(int value)
{
    m_pSensorScanWidget1->SetScale(static_cast<qreal>(value));
}

void MainWindow::SlotOnRequestMapButton()
{
	if (IsConnected())
	{
		SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_MAP, 0, nullptr);
		SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_LOC, 0, nullptr);
	}
}

void MainWindow::SlotOnAutoUpdateMapButton(bool checked)
{
}

void MainWindow::SlotOnResetLocalizerButton()
{
	if (IsConnected())
	{
		SendMessage(ControlPanelProtocol::MessageType::MSG_RESET_LOCALIZER, 0, nullptr);
		SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_MAP, 0, nullptr);
		SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_LOC, 0, nullptr);
	}
}

void MainWindow::SlotOnEnableLocalizerButton(bool checked)
{
	if (IsConnected())
	{
		char payload = checked ? 1 : 0;
		SendMessage(ControlPanelProtocol::MessageType::MSG_ENABLE_LOCALIZER, 1, &payload);
	}
}

void MainWindow::SlotOnSaveMapButton()
{
	const char strFailedSave[] = "Failed to save file!";

	QString fileNameEdit = ui->SaveMapNameEdit->text();
	std::string fileNameEditStd = fileNameEdit.toStdString();

	if (fileNameEditStd.empty())
		fileNameEditStd = "Map";

	std::string safeFileName;
	if (!FileUtils::FindSafeFileName(fileNameEditStd.data(), ".rmap", safeFileName))
	{
		ui->SaveMapStateLabel->setText(strFailedSave);
		return;
	}

	bool writeSuccess = MapUtils::WriteMapToFile_L2Int(m_Map, safeFileName.data(), Utils::LOG2_PROB_QUANTIZATION);

	if (!writeSuccess)
		ui->SaveMapStateLabel->setText(strFailedSave);
	else
	{
		QString labelText = QString("Saved\n%1").arg(QString::fromStdString(safeFileName));
		ui->SaveMapStateLabel->setText(labelText);
	}
}

void MainWindow::SlotOnPlanPathButton()
{
	if (!IsConnected())
		return;

	QPointF mapCursorPos_m;

	// Bail if no cursor is placed
	if (!m_pMapWidget->GetCursorMarkerPos(mapCursorPos_m))
		return;

	ControlPanelProtocol::PlanPathHeader header;
	header.m_X = (float)mapCursorPos_m.x();
	header.m_Y = (float)mapCursorPos_m.y();
	header.m_Radius = 0.3f;

	SendMessage(ControlPanelProtocol::MessageType::MSG_PLAN_PATH, sizeof(header), (const char*)&header);

	// Request path as well
	SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_PATH, 0, nullptr);
}

void MainWindow::SlotOnLocalizerModeButton()
{
	if (!IsConnected())
		return;

	// Both radio buttons are connected here
	// Which one is pressed?

	Localizer::Mode newMode = {};
	if (ui->LocMode0Button->isChecked())
		newMode = Localizer::Mode::LOC_AND_MAP;
	else if (ui->LocMode1Button->isChecked())
		newMode = Localizer::Mode::COMMON_LOC;
	else
		return; // Makes no sense, something must be checked

	uint8_t payload = static_cast<uint8_t>(newMode);
	SendMessage(ControlPanelProtocol::MessageType::MSG_SWITCH_LOCALIZER_MODE, 1, (const char*)&payload);
}
