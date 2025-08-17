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
#include <QtQml>

#include "../Common/ControlPanelProtocol.h"
#include "../Common/Sensor.h"
#include "../Common/Common.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_Socket(this)
{
    ui->setupUi(this);

    // Button events from UI
    QObject::connect(&m_Socket, &QTcpSocket::connected, this, &MainWindow::SlotOnSocketConnected);
    QObject::connect(&m_Socket, &QTcpSocket::disconnected, this, &MainWindow::SlotOnSocketDisconnected);
    QObject::connect(&m_Socket, &QTcpSocket::errorOccurred, this, &MainWindow::SlotOnSocketError);
    QObject::connect(&m_Socket, &QTcpSocket::readyRead, this, &MainWindow::SlotOnSocketReadyRead);

    // Message decoder
    m_MsgDecoder.BindOnReceivedMessage(
        [this](uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
        { OnProtocolNewMessage(msgType, msgPayload, msgPayloadSize); }
    );
    m_MsgDecoder.BindOnError(
        [this](uint8_t errorCode, unsigned int extraData)
        { OnProtocolError(errorCode, extraData); }
    );

    // The graph view
    ui->QuickWidget->setSource(QUrl("qrc:/graph.qml"));
    ui->QuickWidget->show();
    m_pScatterSeries = dynamic_cast<QScatterSeries*>(
        QQmlEngine::contextForObject(ui->QuickWidget->rootObject())->objectForName("scatterSeries"));
}

MainWindow::~MainWindow()
{
    // Disconnect from socket events, otherwise SlotOnSocketDisconnected will be invoked after MainWindow is destroyed!
    m_Socket.disconnect(this);

    delete ui;
}

void MainWindow::SlotOnSocketConnected()
{
    m_MsgDecoder.Reset();
    ui->ConnStateLabel->setText("Connected");
    //m_Socket.write("Hello from RClient!");
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
    while(m_Socket.bytesAvailable())
    {
        qint64 nBytesRead = m_Socket.read(readBuffer, sizeof(readBuffer));
        m_MsgDecoder.ReadStream(readBuffer, nBytesRead);
    }
}

//------------------------------------------------------------------------------------------------------------------------
// Processing of messages

void MainWindow::OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
{
    //QMessageBox(QMessageBox::Icon icon, const QString &title, const QString &text,
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

        case ControlPanelProtocol::MessageType::MSG_SENSOR_SCAN_DATA:
        {
            uint32_t nSamples = *reinterpret_cast<uint32_t*>(msgPayload);
            Sensor::ScanSample* pScanSamplesReceiveBuffer = reinterpret_cast<Sensor::ScanSample*>(&msgPayload[4]);

            // Deserialize data, same way as it was sent
            QList<Sensor::ScanSample> scanSamples;
            scanSamples.resize(static_cast<qsizetype>(nSamples));
            for (uint i = 0; i < nSamples; i++)
                scanSamples[i] = pScanSamplesReceiveBuffer[i];

            // Convert data to Cartesian form
            QList<QPointF> scanSamplesCart;
            scanSamplesCart.resize(static_cast<qsizetype>(nSamples));

            for (uint i = 0; i < nSamples; i++)
            {
                Sensor::ScanSample& sCart = scanSamples[i];
                float xPos = sCart.m_Dist_m * cosf(sCart.m_Angle_rad);
                float yPos = sCart.m_Dist_m * sinf(sCart.m_Angle_rad);
                scanSamplesCart[i] = QPointF(xPos, yPos);
            }

            m_pScatterSeries->clear();
            m_pScatterSeries->append(scanSamplesCart);

            //QString msgBoxTest = QString::asprintf("Received scan data.\nNumber of samples: %i", nSamples);
            //QMessageBox msgBox(QMessageBox::Icon::Information, "Received data", msgBoxTest);
            //msgBox.exec();
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
    m_Socket.write(reinterpret_cast<const char*>(&header), ControlPanelProtocol::MessageHeader::SIZE_PACKED);

    if (payloadSize != 0 && pPayload != nullptr)
        m_Socket.write(pPayload, payloadSize);

    return true;
}

//------------------------------------------------------------------------------------------------------------------------
// Handling of button events

void MainWindow::SlotOnConnectButtonClicked()
{
    QString addressStr = ui->ConnAddress->text();
    m_Socket.connectToHost(addressStr, ControlPanelProtocol::CONTROL_PANEL_PORT);
    ui->ConnStateLabel->setText("Connecting ...");
}

void MainWindow::SlotOnSendMsgButton()
{
    if (m_Socket.state() == QAbstractSocket::SocketState::ConnectedState)
    {
        char msgText[] = "ABC"; // "Message from Client";
        ControlPanelProtocol::MessageHeader header;
        header.Init(ControlPanelProtocol::MessageType::MSG_PRINT, sizeof(msgText));

        m_Socket.write(reinterpret_cast<const char*>(&header), ControlPanelProtocol::MessageHeader::SIZE_PACKED);
        m_Socket.write(msgText, sizeof(msgText));
    }
}

void MainWindow::SlotOnSendNewlineButton()
{
    if (m_Socket.state() == QAbstractSocket::SocketState::ConnectedState)
    {
        ControlPanelProtocol::MessageHeader header;
        header.Init(ControlPanelProtocol::MessageType::MSG_TEST0, 0);
        m_Socket.write(reinterpret_cast<const char*>(&header), ControlPanelProtocol::MessageHeader::SIZE_PACKED);
    }
}

void MainWindow::SlotOnRequestScanDataButton()
{
    if (m_Socket.state() == QAbstractSocket::SocketState::ConnectedState)
    {
        ControlPanelProtocol::MessageHeader header;
        header.Init(ControlPanelProtocol::MessageType::MSG_RQ_SENSOR_SCAN_DATA, 0);

        m_Socket.write(reinterpret_cast<const char*>(&header), ControlPanelProtocol::MessageHeader::SIZE_PACKED);
    }
}

void MainWindow::SlotOnIcpTestStoreButton()
{
    // Store one scan for future comparison
    char payload = 0;
    if (m_Socket.state() == QAbstractSocket::SocketState::ConnectedState)
    {
        SendMessage(ControlPanelProtocol::MessageType::MSG_ICP_TEST, 1, &payload);
    }
}

void MainWindow::SlotOnIcpTestRunButton()
{
    // Store another scan and run ICP
    char payload = 1;
    if (m_Socket.state() == QAbstractSocket::SocketState::ConnectedState)
    {
        SendMessage(ControlPanelProtocol::MessageType::MSG_ICP_TEST, 1, &payload);
    }
}
