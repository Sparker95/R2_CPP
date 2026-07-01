#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "../Common/ControlPanelProtocol.h"
#include "../Common/Map.h"

#include "SensorDataFileWriter.h"

#include <QMainWindow>
#include <QTcpSocket>
#include <QUdpSocket>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class QScatterSeries;
class SensorScanWidget;
class MapWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    ControlPanelProtocol::MessageDecoder m_MsgDecoder;
    QTcpSocket m_ControlPanelSocket;
    QUdpSocket m_ControlInputsSocket;
    SensorDataFileWriter m_SensorFileWriter;
	OccupancyMap m_Map; // Map data
    int m_iControlInputsTimerId = 0;
	bool m_WaitingMap = false;

    // Other UI elements
    QScatterSeries* m_pScatterSeries = nullptr;
    SensorScanWidget* m_pSensorScanWidget0 = nullptr;
    SensorScanWidget* m_pSensorScanWidget1 = nullptr;
	MapWidget* m_pMapWidget = nullptr;

    // Handling communications
    void OnProtocolNewMessage(uint8_t msgType, char* msgPayload, unsigned int msgPayloadSize);
    void OnProtocolError(uint8_t errorCode, unsigned int extraData);
    bool SendMessage(ControlPanelProtocol::MessageType msgType, unsigned int payloadSize, const char* pPayload);
	bool IsConnected() const;

    void timerEvent(QTimerEvent *event) override;

    void UpdateSensorFileStatsLabel();

private slots:
    void SlotOnConnectButtonClicked();
	void SlotOnEnableSensorStreamButton(bool checked = false);
	void SlotOnEnableSensorMotorButton(bool checked = false);
    void SlotOnSaveSensorStreamToFileCheck(bool checked);

	void SlotOnRequestMapButton();
	void SlotOnAutoUpdateMapButton(bool checked = false);
	void SlotOnSaveMapButton();

	void SlotOnResetLocalizerButton();
	void SlotOnEnableLocalizerButton(bool checked = false);
	void SlotOnLocalizerModeButton(); // Used for both radio buttons

	void SlotOnMoveButton();

	void SlotOnEnableAutoControlsButton(bool checked = false);

    void SlotOnSensorScaleValueChanged0(int value);
    void SlotOnSensorScaleValueChanged1(int value);

    void SlotOnSocketConnected();
    void SlotOnSocketDisconnected();
    void SlotOnSocketError();
    void SlotOnSocketReadyRead();

};
#endif // MAINWINDOW_H
