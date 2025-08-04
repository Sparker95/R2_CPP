#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "../Common/ControlPanelProtocol.h"

#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_Socket(this)
{
    ui->setupUi(this);

    QObject::connect(&m_Socket, &QTcpSocket::connected, this, &MainWindow::SlotOnSocketConnected);
    QObject::connect(&m_Socket, &QTcpSocket::disconnected, this, &MainWindow::SlotOnSocketDisconnected);
    QObject::connect(&m_Socket, &QTcpSocket::errorOccurred, this, &MainWindow::SlotOnSocketError);
    QObject::connect(&m_Socket, &QTcpSocket::readyRead, this, &MainWindow::SlotOnSocketReadyRead);

    m_MsgDecoder.BindOnReceivedMessage(
        [this](uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize)
        { OnProtocolNewMessage(msgType, msgPayload, msgPayloadSize); }
    );
    m_MsgDecoder.BindOnError(
        [this](uint8_t errorCode, unsigned int extraData)
        { OnProtocolError(errorCode, extraData); }
    );
}

MainWindow::~MainWindow()
{
    delete ui;
}

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

    default: break;
    }
}

void MainWindow::OnProtocolError(uint8_t errorCode, unsigned int extraData)
{
    QMessageBox msgBox(QMessageBox::Icon::Critical, "Error", "Protocol error");
    msgBox.exec();
}
