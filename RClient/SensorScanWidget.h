#ifndef SENSORSCANWIDGET_H
#define SENSORSCANWIDGET_H

#include <QWidget>

#include "../Common/Sensor.h"
#include "../Common/Chassis.h"

class SensorScanWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SensorScanWidget(QWidget *parent = nullptr);

    void SetScale(qreal maxRange);
    void SetScanData(const QList<Sensor::ScanSample>& scanPolar);

protected:
    void paintEvent(QPaintEvent *event) override;

    QList<Sensor::ScanSample> m_ScanPolar;
    Chassis::ChassisConfig m_ChassisConfig; // Needed for chassis geometry

    qreal m_Scale = 3.0; // Range in meters
signals:
};

#endif // SENSORSCANWIDGET_H
