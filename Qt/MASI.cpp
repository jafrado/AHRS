#include "MASI.h"
#include <QPainter>
#include <QPainterPath>
#include <QConicalGradient>
#include <QRadialGradient>
#include <QSurfaceFormat>
#include <QtMath>
#include <cstdio>

// Dial constants
static const double START_ANGLE = 225.0;   // degrees CW from 12-o'clock: speed = 0 at 7:30
static const double SPAN_ANGLE  = 270.0;   // total clockwise sweep in degrees

// -------------------------------------------------------------------
MASI::MASI(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_size(200)
    , m_offset(2)
    , m_speed(0.0)
    , m_mach(0.0)
    , m_units(Metric)
    , m_fontFamily("Arial")
    , m_labelSize(8)
    , m_labelColor(Qt::white)
    , m_displayScale(1.0)
{
    QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
    fmt.setStencilBufferSize(8);
    setFormat(fmt);

    setMinimumSize(200, 200);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect(this, SIGNAL(widgetReplot()), this, SLOT(widgetReplot_slot()));
}

MASI::~MASI() {}

void MASI::widgetReplot_slot()   { update(); }

void MASI::setSpeed(double speed)
{
    double speedMax = (m_units == Imperial) ? 3000.0 : 1000.0;
    m_speed = qBound(0.0, speed, speedMax);
    emit widgetReplot();
}

void MASI::setMach(double mach)
{
    m_mach = qIsFinite(mach) ? qMax(0.0, mach) : 0.0;
    emit widgetReplot();
}

void MASI::setUnits(Units u)
{
    m_units = u;
    emit widgetReplot();
}

void MASI::setDisplayScale(double scale)
{
    m_displayScale = scale;
    emit widgetReplot();
}

void MASI::setInstrFont(const QFont &font)
{
    m_fontFamily = font.family();
    if (font.pointSize() > 0)
        m_labelSize = font.pointSize();
    emit widgetReplot();
}

void MASI::setLabelColor(const QColor &color)
{
    m_labelColor = color;
    emit widgetReplot();
}

void MASI::resizeEvent(QResizeEvent *event)
{
    m_size = qMin(width(), height()) - 2 * m_offset;
    QOpenGLWidget::resizeEvent(event);
}

// -------------------------------------------------------------------
void MASI::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Dark surround (widget area outside the bezel)
    painter.fillRect(rect(), QColor(0x11, 0x11, 0x11));

    double r  = m_size / 2.0;
    int    bw = qMax(3, (int)r / 30);    // bezel band half-width
    double rI = r - 2.0 * bw;           // bezel inner edge = tick outer edge

    painter.translate(width() / 2.0, height() / 2.0);

    // ------------------------------------------------------------------
    // Black face
    // ------------------------------------------------------------------
    {
        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::black);
        painter.drawEllipse(QPointF(0, 0), rI, rI);
    }

    // ------------------------------------------------------------------
    // Thin inner ring -- separates face from scale area (like SI-800)
    // ------------------------------------------------------------------
    {
        QPen ringPen(QColor(80, 80, 80), 1);
        painter.setPen(ringPen);
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(QPointF(0, 0), rI * 0.82, rI * 0.82);
    }

    // ------------------------------------------------------------------
    // Tick marks and scale labels
    // ------------------------------------------------------------------
    {
        // Imperial: 0–3000 ft/s, 30 ticks × 100 ft/s each; major every 1000, medium every 500
        // Metric:   0–1000 m/s, 100 ticks × 10 m/s each;  major every 100,  medium every 50
        const bool   isImperial = (m_units == Imperial);
        const double speedMax   = isImperial ? 3000.0 : 1000.0;
        const int    nTicks     = isImperial ? 30 : 100;
        const int    majorEvery = isImperial ? 10 : 10;   // every 10 ticks = 1000 or 100 units
        const int    medEvery   = isImperial ?  5 :  5;   // every  5 ticks =  500 or  50 units
        const double angStep    = SPAN_ANGLE / nTicks;

        int    fontSize    = qMax(6, qRound(m_labelSize * m_displayScale));
        QFont  labelFont(m_fontFamily, fontSize);
        QFontMetrics fm(labelFont);
        int    lh = fm.height() + 2;
        int    lw = fm.horizontalAdvance("3000") + 6;

        double majorLen = rI * 0.17;
        double medLen   = rI * 0.11;
        double minorLen = rI * 0.06;

        // Labels sit just inside the major tick inner end
        double labelRad = rI - majorLen - fm.height() * 0.5 - 4;

        QPen  majorPen(Qt::white, 2);
        QPen  thinPen(Qt::white, 1);

        for (int i = 0; i <= nTicks; i++) {
            double speed    = i * (speedMax / nTicks);
            double angle    = START_ANGLE + i * angStep;  // CW from 12-o'clock
            double angleRad = qDegreesToRadians(angle);

            bool isMajor  = (i % majorEvery == 0);
            bool isMedium = (i % medEvery   == 0 && !isMajor);

            // Tick: rotate to angle, draw inward from rI
            painter.save();
            painter.rotate(angle);
            double tickLen;
            if (isMajor) {
                painter.setPen(majorPen);
                tickLen = majorLen;
            } else if (isMedium) {
                painter.setPen(majorPen);
                tickLen = medLen;
            } else {
                painter.setPen(thinPen);
                tickLen = minorLen;
            }
            painter.drawLine(QPointF(0, -rI), QPointF(0, -rI + tickLen));
            painter.restore();

            // Upright label at major ticks, and at medium ticks in Imperial mode
            if (isMajor || (isMedium && isImperial)) {
                double lx = labelRad * sin(angleRad);
                double ly = -labelRad * cos(angleRad);
                painter.setFont(labelFont);
                painter.setPen(QPen(m_labelColor));
                painter.drawText(
                    QRectF(lx - lw / 2.0, ly - lh / 2.0, lw, lh),
                    Qt::AlignCenter,
                    QString::number((int)speed));
            }
        }
    }

    // ------------------------------------------------------------------
    // Units label -- small, at 6-o'clock inside the scale ring
    // ------------------------------------------------------------------
    {
        const char *uStr  = (m_units == Imperial) ? "Ft/Sec" : "m/s";
        int  uFontSz = qMax(5, qRound(m_labelSize * m_displayScale * 1.5));
        QFont uFont(m_fontFamily, uFontSz);
        QFontMetrics ufm(uFont);
        int uw = ufm.horizontalAdvance(uStr) + 4;
        int uh = ufm.height() + 2;
        // Position between needle pivot and lower scale ring
        double uy = rI * 0.58;
        painter.setFont(uFont);
        painter.setPen(QPen(QColor(150, 150, 150)));
        painter.drawText(QRectF(-uw / 2.0, uy, uw, uh), Qt::AlignCenter, uStr);
    }

    // ------------------------------------------------------------------
    // Needle
    // ------------------------------------------------------------------
    {
        double fraction  = qBound(0.0, m_speed / ((m_units == Imperial) ? 3000.0 : 1000.0), 1.0);
        double angle     = START_ANGLE + fraction * SPAN_ANGLE;

        painter.save();
        painter.rotate(angle);

        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::white);

        double tipY  = -(rI - 5.0);
        double baseY =  r / 7.0;        // short counter-tail
        double hw    = qMax(2.5, r / 48.0);

        QPointF pts[3] = {
            QPointF(0,   tipY),
            QPointF(-hw, baseY),
            QPointF(hw,  baseY),
        };
        painter.drawPolygon(pts, 3);

        painter.restore();

        // Pivot cap
        double pivR = qMax(4.0, r / 18.0);
        painter.setPen(QPen(QColor(180, 180, 180), 1));
        painter.setBrush(QColor(40, 40, 40));
        painter.drawEllipse(QPointF(0, 0), pivR, pivR);
    }

    // ------------------------------------------------------------------
    // Mach display box -- centre, orange digits on dark panel
    // ------------------------------------------------------------------
    {
        char buf[32];
        sprintf(buf, "%.3f", m_mach);
        QString machStr = buf;

        // "MACH" label font (small, grey)
        int  lFontSz = qMax(5, qRound(m_labelSize * m_displayScale * 0.70));
        QFont lFont(m_fontFamily, lFontSz);
        QFontMetrics lfm(lFont);

        // Value font (larger, orange)
        int  vFontSz = qMax(8, qRound(m_labelSize * m_displayScale * 1.25));
        QFont vFont(m_fontFamily, vFontSz);
        QFontMetrics vfm(vFont);

        const int padV = 4, padH = 10;
        int boxW = vfm.horizontalAdvance("0.000") + 2 * padH;
        int boxH = lfm.height() + vfm.height() + 3 * padV;
        int boxX = -boxW / 2;
        // Place box slightly above pivot (needle pivot sits at center, box above it)
        int boxY = -boxH / 2 - (int)(rI * 0.12);

        // Dark panel
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(18, 18, 18, 235));
        painter.drawRoundedRect(boxX, boxY, boxW, boxH, 5, 5);

        // Border
        painter.setPen(QPen(QColor(70, 70, 70), 1));
        painter.setBrush(Qt::NoBrush);
        painter.drawRoundedRect(boxX, boxY, boxW, boxH, 5, 5);

        // "MACH" label
        painter.setFont(lFont);
        painter.setPen(QPen(QColor(170, 170, 170)));
        painter.drawText(
            QRectF(boxX, boxY + padV, boxW, lfm.height()),
            Qt::AlignCenter, "MACH");

        // Mach value in amber/orange -- matches the SI-800 rolling-drum window colour
        painter.setFont(vFont);
        painter.setPen(QPen(QColor(255, 155, 0)));
        painter.drawText(
            QRectF(boxX, boxY + padV + lfm.height() + padV, boxW, vfm.height()),
            Qt::AlignCenter, machStr);
    }

    // ------------------------------------------------------------------
    // 3D Metal Bezel -- drawn last so it sits on top of all content
    // ------------------------------------------------------------------
    {
        double rO   = r;
        double rMid = rO - bw;

        QPainterPath ring;
        ring.addEllipse(QPointF(0, 0), rO, rO);
        ring.addEllipse(QPointF(0, 0), rI, rI);
        ring.setFillRule(Qt::OddEvenFill);

        // Layer 1: anisotropic metallic sheen (titanium finish)
        QConicalGradient cg(QPointF(0, 0), 90.0);
        cg.setColorAt(0.000, QColor(218, 226, 232));
        cg.setColorAt(0.060, QColor(172, 180, 186));
        cg.setColorAt(0.120, QColor(115, 122, 129));
        cg.setColorAt(0.220, QColor(84,  90,  97));
        cg.setColorAt(0.380, QColor(68,  74,  80));
        cg.setColorAt(0.500, QColor(58,  64,  70));
        cg.setColorAt(0.620, QColor(68,  74,  80));
        cg.setColorAt(0.780, QColor(84,  90,  97));
        cg.setColorAt(0.880, QColor(115, 122, 129));
        cg.setColorAt(0.940, QColor(172, 180, 186));
        cg.setColorAt(1.000, QColor(218, 226, 232));

        painter.setPen(Qt::NoPen);
        painter.setBrush(cg);
        painter.drawPath(ring);

        // Layer 2: curved-surface highlight
        QRadialGradient sheen(QPointF(0, -rMid * 0.55), rO * 1.7);
        sheen.setColorAt(0.0, QColor(255, 255, 255, 40));
        sheen.setColorAt(0.5, QColor(210, 218, 225, 14));
        sheen.setColorAt(1.0, QColor(0,   0,   0,  28));

        painter.setBrush(sheen);
        painter.drawPath(ring);

        // Outer dark shadow rim
        QPen rimPen(QColor(38, 42, 46));
        rimPen.setWidth(2);
        painter.setPen(rimPen);
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(QPointF(0, 0), rO, rO);

        // Inner chamfer highlight
        QPen chamferPen(QColor(195, 208, 218, 200));
        chamferPen.setWidth(1);
        painter.setPen(chamferPen);
        painter.drawEllipse(QPointF(0, 0), rI, rI);
    }
}
