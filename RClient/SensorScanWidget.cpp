#include "SensorScanWidget.h"

#include <cmath>

#include <QPainter>
#include <QPainterStateGuard>

#include "../Common/CommonIntTypes.h"

// Scan data for testing purposes
const float g_TestScan[] = {
    0.158443, 1.596500, 0.102538, 1.640500, 0.074449, 1.639500, 0.046088, 1.622750, 0.018544, 1.604750, 6.272822, 1.610250,
    6.245279, 1.600250, 6.216645, 1.588000, 6.159922, 1.570250, 6.132378, 1.547500, 6.104016, 1.552750, 5.937938, 1.704250,
    5.909576, 1.712250, 5.606052, 2.622750, 5.520149, 2.220250, 5.492333, 2.196250, 5.406703, 1.962250, 5.351343, 2.053750,
    5.323255, 2.010750, 5.294620, 1.974000, 5.266259, 1.943250, 5.237079, 1.911250, 5.209535, 1.880250, 5.180901, 1.848250,
    5.152267, 1.819750, 5.124178, 1.796500, 5.096634, 1.778250, 5.068545, 1.766500, 5.039366, 1.747750, 5.011277, 1.727250,
    4.983460, 1.713500, 4.893194, 1.189000, 4.832926, 1.028250, 4.805655, 1.028500, 4.778384, 1.031750, 4.751114, 1.036250,
    4.723025, 1.040000, 4.639576, 1.052500, 4.610669, 1.066250, 4.583671, 1.072000, 4.555037, 1.080250, 4.527221, 1.093000,
    4.340689, 1.791250, 4.312327, 1.813250, 4.285056, 1.834000, 4.256968, 1.854750, 4.229424, 1.886750, 4.201608, 1.916000,
    4.022711, 0.211750, 3.834816, 1.529500, 3.750822, 1.508500, 3.419755, 1.691500, 3.308217, 0.604000, 3.281492, 0.602250,
    3.256130, 0.611500, 3.226405, 0.618500, 3.120594, 0.703250, 3.058690, 1.618000, 3.030874, 1.618250, 3.002512, 1.622250,
    2.975241, 1.629250, 2.946880, 1.632500, 2.866704, 0.625250, 2.808889, 1.691000, 2.734440, 0.769500, 2.708806, 0.778250,
    2.682081, 0.793500, 2.656173, 0.813250, 2.589633, 1.863250, 2.507547, 1.956500, 2.480549, 1.998750, 2.321015, 0.287250,
    2.198842, 1.451250, 2.145392, 1.604750, 2.117303, 1.572750, 2.088668, 1.542250, 2.060580, 1.517250, 2.033309, 1.497000,
    2.004947, 1.472250, 1.977676, 1.445000, 1.949860, 1.422000, 1.921226, 1.409000, 1.894773, 1.401000, 1.867230, 1.392750,
    1.840232, 1.385250, 1.811597, 1.377500, 1.784599, 1.369250, 1.756510, 1.361250, 1.729512, 1.355000, 1.701424, 1.348250,
    1.673335, 1.341500, 1.645246, 1.337000, 1.618520, 1.326250, 1.590159, 1.312000, 1.562615, 1.294000, 1.534799, 1.288750,
    1.506983, 1.291750, 1.481348, 1.287000, 1.344449, 1.356250, 1.318269, 1.362750, 1.261819, 1.260250, 1.184370, 1.751000,
    1.157917, 1.781750, 1.103103, 1.814750, 0.998110, 3.278250, 0.915753, 3.340000, 0.804761, 2.966500, 0.718585, 2.070000,
    0.691042, 2.062750, 0.635409, 2.058750, 0.607320, 2.053750, 0.523054, 1.874500, 0.496056, 1.837500, 0.468239, 1.804500,
    0.439878, 1.781250, 0.412607, 1.760000, 0.384245, 1.737250, 0.357247, 1.742750, 0.248982, 2.038500, 0.222530, 2.023000,
    0.194714, 2.006250, 0.167715, 1.994750
};

SensorScanWidget::SensorScanWidget(QWidget *parent)
    : QWidget{parent}
{
    uint nTestSamples = sizeof(g_TestScan) / sizeof(float) / 2;

    /*
    m_ScanPolar.resize(nTestSamples);
    for (uint i = 0; i < nTestSamples; i++)
    {
        m_ScanPolar[i].m_Angle_rad = g_TestScan[2*i];
        m_ScanPolar[i].m_Dist_m = g_TestScan[2*i + 1];
    }
    */
}

void SensorScanWidget::SetScale(qreal maxRange)
{
    m_Scale = maxRange;
    update();
}

void SensorScanWidget::SetScanData(const QList<Sensor::ScanSample>& scanPolar)
{
    m_ScanPolar = scanPolar;
    update();
}

void SensorScanWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    const QColor bgColor(0, 0, 105);
    const QColor rayColor(0, 0, 250); // 150);
    const QColor rayHitColor(185, 0, 92);
    const QColor chassisColor(237, 185, 3);
    const QColor crossColor(166, 166, 255);

    painter.setRenderHint(QPainter::Antialiasing, false);

    // Fill background
    painter.fillRect(0, 0, width(), height(), bgColor);

    QPainterStateGuard Johny(&painter);

    // Coordinate system is aligned with chassis origin, and matematical notation.
    // X is forward, Y is sideways to the left.
    painter.setWindow(-m_Scale, -m_Scale, 2.0 * m_Scale, 2.0 * m_Scale);
    painter.rotate(-90.0);
    painter.scale(1.0, -1.0);

    // Diagnostics: draw X, Y axis
    if (false)
    {
        // X axis
        QPen pen;
        pen.setCosmetic(true); // Ensures that scaling doesn't affect width
        pen.setColor(QColor(255, 0, 0)); // Red
        pen.setWidth(3);
        painter.setPen(pen);
        painter.drawLine(QPointF(0, 0), QPointF(1.0, 0));

        // Y axis
        pen.setColor(QColor(0, 255, 0)); // Green
        painter.setPen(pen);
        painter.drawLine(QPointF(0, 0), QPointF(0, 1.0));
    }

    // Draw rays
    QList<QPointF> rayPosEnds;
    {
        QPen pen;
        pen.setColor(rayColor);
        pen.setWidthF(0.03 * m_Scale);

        painter.setPen(pen);

        QPointF posRayStart(m_ChassisConfig.m_sensorOffsetFwd, m_ChassisConfig.m_sensorOffsetSide);
        uint nSamples = m_ScanPolar.size();
        rayPosEnds.resize(nSamples);

        for (uint i = 0; i < nSamples; i++)
        {
            const Sensor::ScanSample& sample = m_ScanPolar[i];
            float rayDir = sample.m_Angle_rad + m_ChassisConfig.m_sensorOffsetDir_rad;
            QPointF posRayEnd = posRayStart + QPointF(sample.m_Dist_m * std::cos(rayDir),
                                                      sample.m_Dist_m * std::sin(rayDir));
            rayPosEnds[i] = posRayEnd;
        }

        for (QPointF& posRayEnd : rayPosEnds)
            painter.drawLine(posRayStart, posRayEnd);
    }

    // Draw car shape
    {
        QBrush brush(chassisColor);
        painter.setBrush(brush);
        const auto& conf = m_ChassisConfig;
        QRectF rect(-conf.m_lengthBwd, -0.5*conf.m_width, conf.m_lengthFwd + conf.m_lengthBwd, conf.m_width);
        painter.fillRect(rect, brush);
    }

    // Draw ray ends
    {
        QBrush brush(rayHitColor);
        painter.setBrush(brush);
        const qreal hitRadius = 0.035 * m_Scale;
        for (QPointF& posRayEnd : rayPosEnds)
        {
            QRectF rect(posRayEnd.x() - 0.5 * hitRadius, posRayEnd.y() - 0.5 * hitRadius,
                        hitRadius, hitRadius);
            painter.fillRect(rect, brush);
        }
    }


    // Draw cross over origin
    {
        QPen pen;
        pen.setColor(crossColor);
        pen.setCosmetic(true);
        pen.setWidth(1);
        painter.setPen(pen);

        painter.drawLine(QPointF(-1.0, 0), QPointF(1.0, 0));
        painter.drawLine(QPointF(0, -1.0), QPointF(0, 1.0));
    }
}


