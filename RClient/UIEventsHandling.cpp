#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <cmath>

// Generic containers:
#include <QList>
#include <QPointF>

// UI:
#include <QMessageBox>

// Graph related:
#include <QQuickWidget>
#include <QQuickItem>
#include <QQmlContext>
#include <QScatterSeries>

#include "InputHandling.h"
#include "SensorScanWidget.h"

#include "../Common/ControlPanelProtocol.h"
#include "../Common/ControlInputsProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Common.h"

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

void MainWindow::SlotOnEnableSensorStreamButton()
{
    char payload = 1;
    if (m_ControlPanelSocket.state() == QAbstractSocket::SocketState::ConnectedState)
    {
        SendMessage(ControlPanelProtocol::MessageType::MSG_ENABLE_STREAM_SENSOR_DATA, 1, &payload);
    }
}

void MainWindow::SlotOnDisableSensorStreamButton()
{
    char payload = 0;
    if (m_ControlPanelSocket.state() == QAbstractSocket::SocketState::ConnectedState)
    {
        SendMessage(ControlPanelProtocol::MessageType::MSG_ENABLE_STREAM_SENSOR_DATA, 1, &payload);
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
	if (m_ControlPanelSocket.state() == QAbstractSocket::SocketState::ConnectedState)
	{
		SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_MAP, 0, nullptr);
	}
}
