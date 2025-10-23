#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "../Common/ControlPanelProtocol.h"

#include <QMainWindow>
#include <QTcpSocket>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class QScatterSeries;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    ControlPanelProtocol::MessageDecoder m_MsgDecoder;
    QTcpSocket m_Socket;

    // Other UI elements
    QScatterSeries* m_pScatterSeries = nullptr;

    // Handling communications
    void OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize);
    void OnProtocolError(uint8_t errorCode, unsigned int extraData);
    bool SendMessage(ControlPanelProtocol::MessageType msgType, unsigned int payloadSize, const char* pPayload);

private slots:
    void SlotOnConnectButtonClicked();
    void SlotOnSendMsgButton();
    void SlotOnSendNewlineButton();
    void SlotOnRequestScanDataButton();
    void SlotOnIcpTestStoreButton();
    void SlotOnIcpTestRunButton();
    void SlotOnSaveScansButton();

    void SlotOnSocketConnected();
    void SlotOnSocketDisconnected();
    void SlotOnSocketError();
    void SlotOnSocketReadyRead();

};
#endif // MAINWINDOW_H
