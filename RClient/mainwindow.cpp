#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <cmath>

// Generic containers:
#include <QList>
#include <QPointF>

// UI:
#include <QMessageBox>
#include <QGridLayout>

#include "InputHandling.h"
#include "SensorScanWidget.h"
#include "MapWidget.h"

#include "../Common/ControlPanelProtocol.h"
#include "../Common/ControlInputsProtocol.h"
#include "../Common/Chassis.h"
#include "../Common/Sensor.h"
#include "../Common/Common.h"

const uint TIMER_INTERVAL_MS = 40;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
	, m_ControlPanelSocket(this)
	, m_Map(16, 16, 8, 8, 0.1f) // Must have some map data
{
    ui->setupUi(this);

	// UI signals
	QObject::connect(ui->EnableSensorStreamButton, &QPushButton::clicked, this, &MainWindow::SlotOnEnableSensorStreamButton);
	QObject::connect(ui->EnableSensorMotorButton, &QPushButton::clicked, this, &MainWindow::SlotOnEnableSensorMotorButton);
	QObject::connect(ui->AutoResfreshMapButton, &QPushButton::clicked, this, &MainWindow::SlotOnAutoUpdateMapButton);
	QObject::connect(ui->ResetLocalizerButton, &QPushButton::clicked, this, &MainWindow::SlotOnResetLocalizerButton);
	QObject::connect(ui->EnableLocalizerButton, &QPushButton::clicked, this, &MainWindow::SlotOnEnableLocalizerButton);
	QObject::connect(ui->SaveMapButton, &QPushButton::clicked, this, &MainWindow::SlotOnSaveMapButton);

	// Socket signals
    QObject::connect(&m_ControlPanelSocket, &QTcpSocket::connected, this, &MainWindow::SlotOnSocketConnected);
    QObject::connect(&m_ControlPanelSocket, &QTcpSocket::disconnected, this, &MainWindow::SlotOnSocketDisconnected);
    QObject::connect(&m_ControlPanelSocket, &QTcpSocket::errorOccurred, this, &MainWindow::SlotOnSocketError);
    QObject::connect(&m_ControlPanelSocket, &QTcpSocket::readyRead, this, &MainWindow::SlotOnSocketReadyRead);

    // Message decoder
    m_MsgDecoder.BindOnReceivedMessage(
        [this](uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
        { OnProtocolNewMessage(msgType, msgPayload, msgPayloadSize); }
    );
    m_MsgDecoder.BindOnError(
        [this](uint8_t errorCode, unsigned int extraData)
        { OnProtocolError(errorCode, extraData); }
    );

    // Timer for control inputs
	m_iControlInputsTimerId = startTimer(TIMER_INTERVAL_MS, Qt::TimerType::PreciseTimer);

    // Sensor scan widgets
    SensorScanWidget** sensorScanWidgets[] = {&m_pSensorScanWidget0, &m_pSensorScanWidget1};
    QFrame* sensorScanFrames[] = {ui->SensorScanFrame0, ui->SensorScanFrame1};
    for (uint frameId = 0; frameId < 2; frameId++)
    {
        QGridLayout* gridLayout = new QGridLayout();
        *sensorScanWidgets[frameId] = new SensorScanWidget();
        gridLayout->setSpacing(1);
        gridLayout->setContentsMargins(0, 0, 0, 0);
        gridLayout->addWidget(*sensorScanWidgets[frameId], 0, 0);
        sensorScanFrames[frameId]->setLayout(gridLayout);
    }
    // Synchronize values from sliders to scan widgets
    SlotOnSensorScaleValueChanged0(ui->SensorScanScaleSlider0->value());
    SlotOnSensorScaleValueChanged1(ui->SensorScanScaleSlider1->value());

    // Map widget
    {
        QGridLayout* gridLayout = new QGridLayout();
        gridLayout->setSpacing(1);
        gridLayout->setContentsMargins(0, 0, 0, 0);

		m_pMapWidget = new MapWidget();
		gridLayout->addWidget(m_pMapWidget, 0, 0);

        ui->MapFrame->setLayout(gridLayout);

		m_pMapWidget->SetMapData(&m_Map);
    }
}

MainWindow::~MainWindow()
{
    // Disconnect from socket events, otherwise SlotOnSocketDisconnected will be invoked after MainWindow is destroyed!
    m_ControlPanelSocket.disconnect(this);

    delete ui;
}

void MainWindow::SlotOnSocketConnected()
{
    m_MsgDecoder.Reset();
    ui->ConnStateLabel->setText("Connected");

	// Immediately request options from server, to update our UI
	SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_OPTIONS, 0, nullptr);
}

void MainWindow::SlotOnSocketDisconnected()
{
    ui->ConnStateLabel->setText("Disconnected");
}

void MainWindow::SlotOnSocketError()
{
    ui->ConnStateLabel->setText("Socket error");
}

void MainWindow::SlotOnSocketReadyRead()
{
    char readBuffer[1024];
    while(m_ControlPanelSocket.bytesAvailable())
    {
        qint64 nBytesRead = m_ControlPanelSocket.read(readBuffer, sizeof(readBuffer));
        m_MsgDecoder.ReadStream(readBuffer, nBytesRead);
    }
}

void MainWindow::UpdateSensorFileStatsLabel()
{
    if (!m_SensorFileWriter.GetIsOpen())
    {
        ui->StreamFileStateLabel->setText("Stream file: closed");
        return;
    }

    QString fileName = QString::fromStdString(m_SensorFileWriter.GetFileName());
    uint nBytes, nBlocks;
    m_SensorFileWriter.GetStats(nBytes, nBlocks);

    QString labelText = QString("%1\n%2 Blocks, %3 kB").arg(fileName, QString::number(nBlocks), QString::number(nBytes / 1024));

    ui->StreamFileStateLabel->setText(labelText);
}

void MainWindow::timerEvent(QTimerEvent *event)
{
    if (event->timerId() != m_iControlInputsTimerId)
    {
        QMainWindow::timerEvent(event);
        return;
    }

	// Send control inputs values to server
	if (IsConnected())
	{
		using TMoveSpeed = ControlInputsProtocol::Message::MoveSpeed;
		using TMessage = ControlInputsProtocol::Message;

		ControlInputsProtocol::Message msg;

		// Handle inputs
		bool moveDirFwd = false;
		TMoveSpeed moveSpeed = {};
		bool move = InputHandling::GetMoveInput(moveSpeed, moveDirFwd);
		float steerInputNorm = InputHandling::GetSteerInputNorm();
		bool brakeInput = InputHandling::GetBrakeInput();

		if (brakeInput)
		{
			msg.m_CmdFlags |= TMessage::CMD_FLAG_BRAKE;
		}
		else if (move)
		{
			msg.m_CmdFlags |= TMessage::CMD_FLAG_MOVE;
			msg.m_MoveSpeed = moveSpeed;
			if (moveDirFwd)
				msg.m_CmdFlags |= TMessage::CMD_FLAG_MOVE_DIR_FWD;
		}

		msg.m_fSteerNorm = steerInputNorm;

		QByteArray dataArray(reinterpret_cast<const char*>(&msg), sizeof(ControlInputsProtocol::Message));
		QHostAddress receiverAddr = m_ControlPanelSocket.peerAddress();
		m_ControlInputsSocket.writeDatagram(dataArray, receiverAddr, ControlInputsProtocol::CONTROL_INPUTS_PORT);

		// Set label text
		QString labelText = QString::asprintf("Move: %i\nMove Dir: %i\nMove Speed: %i\nBrake: %i\nSteer: %.2f",
											  move,
											  moveDirFwd,
											  static_cast<unsigned int>(moveSpeed),
											  static_cast<int>(brakeInput),
											  steerInputNorm);
		ui->ControlInputsLabel->setText(labelText);
	}

	// Request map refresh
	static uint s_MapUpdateTimer = 0;
	const uint MAP_UPDATE_INTERVAL = 1 * 1000 / TIMER_INTERVAL_MS;
	{
		s_MapUpdateTimer++;
		if (s_MapUpdateTimer > MAP_UPDATE_INTERVAL)
		{
			s_MapUpdateTimer = 0;

			if (ui->AutoResfreshMapButton->isChecked() && IsConnected())
			{
				SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_MAP, 0, nullptr);
				SendMessage(ControlPanelProtocol::MessageType::MSG_REQUEST_LOC, 0, nullptr);
			}
		}
	}
}

//------------------------------------------------------------------------------------------------------------------------
// Processing of messages

void MainWindow::OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
{
    //QMessageBox(QMessageBox::Icon icon, const QString &title, const QString &text,
    ControlPanelProtocol::BufferReader reader(msgPayload, msgPayloadSize);

    switch (static_cast<ControlPanelProtocol::MessageType>(msgType))
    {
        case ControlPanelProtocol::MessageType::MSG_PRINT:
        {
            // For safety, null terminate string
            if (msgPayloadSize < ControlPanelProtocol::MAX_PAYLOAD_SIZE - 1)
                msgPayload[msgPayloadSize] = 0;
            else
                msgPayload[ControlPanelProtocol::MAX_PAYLOAD_SIZE - 1] = 0;

            QMessageBox msgBox(QMessageBox::Icon::Information, "Message from server", msgPayload);
            msgBox.exec();
            break;
        }

        case ControlPanelProtocol::MessageType::MSG_SENSOR_DATA:
        {
            // Deserialize data, same way as it was sent
            ControlPanelProtocol::SensorDataHeader header = reader.Read<ControlPanelProtocol::SensorDataHeader>();

            QList<Sensor::ScanSampleInt> intScanSamples;
            intScanSamples.resize(static_cast<qsizetype>(header.m_nSamples));
            for (uint i = 0; i < header.m_nSamples; i++)
                intScanSamples[i] = reader.Read<Sensor::ScanSampleInt>();

            // Convert scan samples from int to float
            QList<Sensor::ScanSample> scanSamples;
            scanSamples.resize(intScanSamples.size());
            for (uint i = 0; i < header.m_nSamples; i++)
            {
                Sensor::ScanSample& sample = scanSamples[i];
                Sensor::ScanSampleInt& sampleInt = intScanSamples[i];
                Sensor::ConvertSampleIntToFloat(sampleInt, sample);
            }

            m_pSensorScanWidget0->SetScanData(scanSamples);
            m_pSensorScanWidget1->SetScanData(scanSamples);

            // Convert data to Cartesian form
            QList<QPointF> scanSamplesCart;
            scanSamplesCart.resize(static_cast<qsizetype>(header.m_nSamples));

            for (uint i = 0; i < header.m_nSamples; i++)
            {
                Sensor::ScanSample& sCart = scanSamples[i];
                float angleDisplay = sCart.m_Angle_rad + PI;
                float xPos = sCart.m_Dist_m * cosf(angleDisplay);
                float yPos = sCart.m_Dist_m * sinf(angleDisplay);
                scanSamplesCart[i] = QPointF(xPos, yPos);
            }

            // Write to file
            if (m_SensorFileWriter.GetIsOpen())
            {
                m_SensorFileWriter.WriteData(header, scanSamples);
                UpdateSensorFileStatsLabel();
            }

            break;
        }

		case ControlPanelProtocol::MessageType::MSG_MAP:
		{
			ControlPanelProtocol::MapDataHeader header = reader.Read<ControlPanelProtocol::MapDataHeader>();

			// WIP too many allocations and destructions here
			Map m(header.m_xSize, header.m_ySize, header.m_xCenter, header.m_yCenter, header.m_Res_w);
			for (uint y = 0; y < m.GetSize().y; y++)
			{
				for (uint x = 0; x < m.GetSize().x; x++)
					m.SetDataAtPos(Map::TCellPos(x, y), reader.Read<Map::TCellData>());
			}

			m_Map = std::move(m);
			m_pMapWidget->SetMapData(&m_Map);

			break;
		}

		case ControlPanelProtocol::MessageType::MSG_LOC:
		{
			uint nPoses = reader.Read<uint>();

			QList<Pose> poses;
			poses.resize(nPoses);

			for (uint i = 0; i < nPoses; i++)
				poses[i] = reader.Read<Pose>();

			m_pMapWidget->SetLocationData(poses);

			break;
		}

		case ControlPanelProtocol::MessageType::MSG_OPTIONS:
		{
			using TOptions = ControlPanelProtocol::OptionsHeader;

			TOptions opts = reader.Read<TOptions>();

			ui->EnableSensorStreamButton->setChecked(opts.m_Options & TOptions::OPT_STREAM_SENSOR_DATA);
			ui->EnableSensorMotorButton->setChecked(opts.m_Options & TOptions::OPT_SENSOR_MOTOR);
			ui->EnableLocalizerButton->setChecked(opts.m_Options & TOptions::OPT_UPDATE_LOCALIZER);

			break;
		}

        default: break;
    }
}

void MainWindow::OnProtocolError(uint8_t errorCode, unsigned int extraData)
{
    QMessageBox msgBox(QMessageBox::Icon::Critical, "Error", "Protocol error");
    msgBox.exec();
}

bool MainWindow::SendMessage(ControlPanelProtocol::MessageType msgType, unsigned int payloadSize, const char* pPayload)
{
    ControlPanelProtocol::MessageHeader header;
    header.Init(msgType, payloadSize);

    // Send header
    m_ControlPanelSocket.write(reinterpret_cast<const char*>(&header), ControlPanelProtocol::MessageHeader::SIZE_PACKED);

    if (payloadSize != 0 && pPayload != nullptr)
        m_ControlPanelSocket.write(pPayload, payloadSize);

    return true;
}


bool MainWindow::IsConnected() const
{
	return m_ControlPanelSocket.state() == QAbstractSocket::SocketState::ConnectedState &&
		m_ControlInputsSocket.state() == QAbstractSocket::SocketState::BoundState;
}
