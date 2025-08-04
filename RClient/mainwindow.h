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

    void OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize);
    void OnProtocolError(uint8_t errorCode, unsigned int extraData);

private slots:
    void SlotOnConnectButtonClicked();
    void SlotOnSendMsgButton();
    void SlotOnSendNewlineButton();

    void SlotOnSocketConnected();
    void SlotOnSocketDisconnected();
    void SlotOnSocketError();
    void SlotOnSocketReadyRead();

};
#endif // MAINWINDOW_H
