/***
 * Widgets for AHRS Display
 * AHRSAttitudeIndicator derived from qAttitudeIndicator by cmex81/qAttitudeIndicator
 * cmex81/qAttitudeIndicator
 * https://github.com/cmex81/qAttitudeIndicator
 *
 * AHRSAttitude Indicator - enhanced by bushuhui
 * AHRSCompass - derived from bushui/qFlightInstruments QCompass
 * bushuhui/qFlightInstruments
 * https://github.com/bushuhui/qFlightInstruments
 *
 * AHRSInfo RTK GPS Indicator - by jafrado
 *
 */
#include "ahrs_widgets.h"

void AHRSAttitudeIndicator::widgetReplot_slot(void)
{
    update();
}

//from cmex81/bushuhui
AHRSAttitudeIndicator::AHRSAttitudeIndicator(QWidget* parent)
    : QOpenGLWidget(parent)
{
    // Request a stencil buffer -- QPainter needs it for QRegion::Ellipse clipping
    // (setClipRegion with non-rectangular shapes uses the stencil buffer on OpenGL)
    QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
    fmt.setStencilBufferSize(8);
    setFormat(fmt);

    //QTimer *timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    //timer->start(500);

    //connect(this, SIGNAL(widgetReplot(void)), this, SLOT(widgetReplot_slot(void)));
    offset = 2;
    size = 180;
    setMinimumSize(180, 180);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    resize(size, size);
    setFocusPolicy(Qt::NoFocus);
    roll = 0.0;
    pitch = 0.0;
    m_fontFamily   = "Arial";
    m_labelSize    = 8;
    m_labelColor   = Qt::white;
    m_displayScale = 1.0;
}

AHRSAttitudeIndicator::~AHRSAttitudeIndicator()
{}

void AHRSAttitudeIndicator::resizeEvent(QResizeEvent* event)
{
    size = qMin(width(), height()) - 2 * offset;
    QOpenGLWidget::resizeEvent(event);  // resizes the FBO to match the new widget size
}

void AHRSAttitudeIndicator::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    // Clear entire widget -- QOpenGLWidget does not auto-clear between frames
    painter.fillRect(rect(), QColor(0x11, 0x11, 0x11));

    QPoint center(0, 0);
    QPen whitePen(Qt::white);
    QPen blackPen(Qt::black);
    QBrush bgSky(QColor(48, 172, 220));
    QBrush bgGround(QColor(247, 168, 21));

    QPen   pitchPen(Qt::white);
    QPen   pitchZero(Qt::green);
    pitchZero.setWidth(2);
    whitePen.setWidth(2);
    blackPen.setWidth(1);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(width() / 2, height() / 2);

    int side = qMin(width(), height());
    painter.scale(side / (qreal)(size), side / (qreal)(size));
    painter.setPen(blackPen);
    painter.rotate(roll);
    painter.setBrush(bgSky);

    // FIXME: AHRS output left-hand values
    double pitch_tem = -pitch;

    // Clip to circle BEFORE drawing the sky/ground chords.
    // Anti-aliased drawChord edges bleed yellow/orange pixels outside the
    // mathematical ellipse boundary; clipping first prevents those artifacts.
    // QPainterPath respects the logical transform (scale + rotate) on
    // QOpenGLWidget; QRegion::Ellipse + setClipRegion does not.
    //
    // The clip is inset by a fixed 3 px -- just enough that its AA fringe falls
    // inside the bezel band and is fully covered.  bezelBw drives the ring width
    // only; it is NOT the clip inset (decoupled so wider bezels don't clip content).
    int bezelBw = qMax(3, (size / 2) / 30);   // safe max: markers sit at r - 2 - max(4,r/15)
    QPainterPath clipPath;
    clipPath.addEllipse(QPointF(0, 0), size / 2.0 - 3, size / 2.0 - 3);
    painter.setClipPath(clipPath);

    // draw background
    {
        int y_min, y_max;

        y_min = size / 2 * -40.0 / 45.0;
        y_max = size / 2 * 40.0 / 45.0;

        int y = size / 2 * pitch_tem / 45.0;
        if (y < y_min) y = y_min;
        if (y > y_max) y = y_max;

        int x = sqrt(size * size / 4 - y * y);
        qreal gr = atan((double)(y) / x);
        gr = gr * 180.0 / M_PI;

        painter.setPen(blackPen);
        painter.setBrush(bgSky);
        painter.drawChord(-size / 2, -size / 2, size, size,
            gr * 16, (180 - 2 * gr) * 16);

        painter.setBrush(bgGround);
        painter.drawChord(-size / 2, -size / 2, size, size,
            gr * 16, -(180 + 2 * gr) * 16);
    }


    // draw pitch lines & marker
    {
        int x, y, x1, y1;
        int textWidth;
        double p, r;
        int ll = size / 8, l;

        int     fontSize = qMax(4, qRound(m_labelSize * m_displayScale));
        QString s;

        pitchPen.setWidth(2);
        painter.setFont(QFont(m_fontFamily, fontSize));
        QFontMetrics pitchFM(painter.font());


        // draw lines
        for (int i = -9; i <= 9; i++) {
            p = i * 10;

            s = QString("%1").arg(-p);

            if (i % 3 == 0)
                l = ll;
            else
                l = ll / 2;

            if (i == 0) {
                painter.setPen(pitchZero);
                l = l * 1.8;
            }
            else {
                painter.setPen(pitchPen);
            }

            y = size / 2 * p / 45.0 - size / 2 * pitch_tem / 45.;
            x = l;

            r = sqrt(x * x + y * y);
            if (r > size / 2) continue;

            painter.drawLine(QPointF(-l, 1.0 * y), QPointF(l, 1.0 * y));

            textWidth = 100;

            if (i % 3 == 0 && i != 0) {
                painter.setPen(QPen(m_labelColor));

                int labelH = pitchFM.height() + 4;
                x1 = -x - 2 - textWidth;
                y1 = y - labelH / 2;
                painter.drawText(QRectF(x1, y1, textWidth, labelH),
                    Qt::AlignRight | Qt::AlignVCenter, s);
            }
        }

        // draw marker
        int     markerSize = size / 20;
        float   fx1, fy1, fx2, fy2, fx3, fy3;
        painter.setBrush(QBrush(Qt::red));
        painter.setPen(Qt::NoPen);

        fx1 = markerSize;
        fy1 = 0;
        fx2 = fx1 + markerSize;
        fy2 = -markerSize / 2;
        fx3 = fx1 + markerSize;
        fy3 = markerSize / 2;

        QPointF points[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(points, 3);

        QPointF points2[3] = {
            QPointF(-fx1, fy1),
            QPointF(-fx2, fy2),
            QPointF(-fx3, fy3)
        };
        painter.drawPolygon(points2, 3);

        // Center tick marks -- black line from tip to base midpoint on each arrow
        QPen tickPen(Qt::black, qMax(1, markerSize / 12));
        painter.setPen(tickPen);
        painter.drawLine(QPointF(fx1, 0), QPointF(fx2, 0));    // right arrow
        painter.drawLine(QPointF(-fx1, 0), QPointF(-fx2, 0));  // left arrow
    }

    // draw roll degree lines
    {
        int     nRollLines = 36;
        float   rotAng = 360.0 / nRollLines;
        int     rollLineLeng = size / 25;
        double  fx1, fy1, fx2, fy2;
        int     fontSize = qMax(4, qRound(m_labelSize * m_displayScale));
        QString s;

        blackPen.setWidth(1);
        painter.setPen(blackPen);
        painter.setFont(QFont(m_fontFamily, fontSize));
        QFontMetrics rollFM(painter.font());

        for (int i = 0; i < nRollLines; i++) {
            if (i < nRollLines / 2)
                s = QString("%1").arg(-i * rotAng);
            else
                s = QString("%1").arg(360 - i * rotAng);

            fx1 = 0;
            fy1 = -(size / 2.0 - 2.0 * bezelBw);   // start at bezel inner edge
            fx2 = 0;

            if (i % 6 == 0) {
                // Major tick (every 60 deg) -- triple length, thick pen
                blackPen.setWidth(2);
                painter.setPen(blackPen);
                fy2 = fy1 + rollLineLeng * 3;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
                blackPen.setWidth(1);

                fy2 = fy1 + rollLineLeng * 3 + 2;
                painter.setPen(QPen(m_labelColor));
                painter.drawText(QRectF(-50, fy2, 100, rollFM.height() + 4),
                    Qt::AlignCenter, s);
                painter.setPen(blackPen);
            }
            else if (i % 3 == 0) {
                // Minor axis (midpoint between 60 deg majors) -- double length
                fy2 = fy1 + rollLineLeng * 2;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));

                fy2 = fy1 + rollLineLeng * 2 + 2;
                painter.setPen(QPen(m_labelColor));
                painter.drawText(QRectF(-50, fy2, 100, rollFM.height() + 4),
                    Qt::AlignCenter, s);
                painter.setPen(blackPen);
            }
            else {
                fy2 = fy1 + rollLineLeng / 2;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
            }

            painter.rotate(rotAng);
        }
    }

    // draw roll marker
    {
        int     bezelBand     = qMax(4, size / 30);  // 2*bw -- keeps tip just inside bezel
        int     rollMarkerSize = size / 25;
        double  fx1, fy1, fx2, fy2, fx3, fy3;

        painter.rotate(-roll);
        painter.setBrush(QBrush(Qt::black));

        fx1 = 0;
        fy1 = -size / 2 + offset + bezelBand;
        fx2 = fx1 - rollMarkerSize / 2;
        fy2 = fy1 + rollMarkerSize;
        fx3 = fx1 + rollMarkerSize / 2;
        fy3 = fy1 + rollMarkerSize;

        QPointF points[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(points, 3);
    }

    // 3D Metal Bezel ring -- drawn last, on top of all instrument content
    {
        painter.setClipping(false);

        double rO   = size / 2.0;
        int    bw   = bezelBw;         // qMax(3, r/30) -- keeps ring inside marker positions
        double rI   = rO - 2.0 * bw;
        double rMid = rO - bw;

        QPainterPath ring;
        ring.addEllipse(QPointF(0, 0), rO, rO);
        ring.addEllipse(QPointF(0, 0), rI, rI);
        ring.setFillRule(Qt::OddEvenFill);

        // Layer 1: Anisotropic metallic sheen -- titanium finish.
        // Bright silver-blue at 12 o'clock, satin mid-gray at the sides,
        // charcoal (not near-black) at 6 o'clock.
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

        // Layer 2: Curved-surface sheen -- radial gradient shifted toward the top
        // simulates the highlight on a rounded/chamfered ring cross-section.
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

void AHRSAttitudeIndicator::keyPressEvent(QKeyEvent* event)
{
    switch (event->key())
    {

    case Qt::Key_Left:
        roll -= 1.0;
        break;
    case Qt::Key_Right:
        roll += 1.0;
        break;
    case Qt::Key_Down:
        if (pitch > -90.0f)
            pitch -= 1.0;
        break;
    case Qt::Key_Up:
        if (pitch < 90.0f)
            pitch += 1.0;
        break;
    default:
        QWidget::keyPressEvent(event);
        break;
    }
    qDebug() << "roll=" << roll << " pitch=" << pitch;
    update();
}

void AHRSAttitudeIndicator::setInstrFont(const QFont &f) {
    m_fontFamily = f.family();
    if (f.pointSize() > 0) m_labelSize = f.pointSize();
    update();
}

void AHRSAttitudeIndicator::setLabelColor(const QColor &c) { m_labelColor = c; update(); }
void AHRSAttitudeIndicator::setDisplayScale(double scale)  { m_displayScale = scale; update(); }

//from bushuhui
AHRSCompass::AHRSCompass(QWidget* parent)
    : QOpenGLWidget(parent)
{
    QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
    fmt.setStencilBufferSize(8);
    setFormat(fmt);

    //QTimer *timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    //timer->start(500);


    //connect(this, SIGNAL(canvasReplot(void)), this, SLOT(canvasReplot_slot(void)));

    sizeMin = 180;
    sizeMax = 600;
    offset = 2;
    size = sizeMin - 2 * offset;

    setMinimumSize(sizeMin, sizeMin);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    resize(sizeMin, sizeMin);

    setFocusPolicy(Qt::NoFocus);

    yaw = 0.0;
    altitude = 0.0;
    msl = 0.0;
    m_fontFamily   = "Arial";
    m_labelSize    = 8;
    m_labelColor   = Qt::black;
    m_displayScale = 1.0;
    m_units        = Metric;
}

AHRSCompass::~AHRSCompass()
{

}


void AHRSCompass::canvasReplot_slot(void)
{
    update();
}

void AHRSCompass::resizeEvent(QResizeEvent* event)
{
    size = qMin(width(), height()) - 2 * offset;
    QOpenGLWidget::resizeEvent(event);  // resizes the FBO to match the new widget size
}

void AHRSCompass::paintEvent(QPaintEvent*)
{

    QPainter painter(this);

    // Clear widget area first -- prevents stale pixels in corners outside the ellipse
    // and eliminates ghost artifacts when the altitude box shifts slightly between frames
    painter.fillRect(rect(), QColor(0x11, 0x11, 0x11));

    QBrush bgGround(QColor(48, 172, 220));

    QPen   whitePen(Qt::white);
    QPen   blackPen(Qt::black);
    QPen   redPen(Qt::red);
    QPen   bluePen(Qt::blue);
    QPen   greenPen(Qt::green);

    whitePen.setWidth(1);
    blackPen.setWidth(2);
    redPen.setWidth(2);
    bluePen.setWidth(2);
    greenPen.setWidth(2);

    painter.setRenderHint(QPainter::Antialiasing);

    painter.translate(width() / 2, height() / 2);


    // draw background
    {
        painter.setPen(blackPen);
        painter.setBrush(bgGround);

        painter.drawEllipse(-size / 2, -size / 2, size, size);
    }


    // Bezel inner radius -- ticks must start here so the bezel doesn't bury them
    int compassBw = qMax(3, (size / 2) / 30);

    // draw yaw lines
    {
        int     nyawLines = 36;
        float   rotAng = 360.0 / nyawLines;
        int     yawLineLeng = size / 25;
        double  fx1, fy1, fx2, fy2;
        int     fontSize = qMax(4, qRound(m_labelSize * m_displayScale));
        QString s;

        blackPen.setWidth(1);
        painter.setPen(blackPen);

        // save()/restore() so the 36 x rotate(-10 deg) calls don't accumulate
        // floating-point error that would cause subsequent drawing (the altitude
        // box etc.) to drift slightly every frame, producing white-edge artifacts
        painter.save();
        for (int i = 0; i < nyawLines; i++) {

            if (i == 0) {
                s = "N";
                painter.setPen(bluePen);

                painter.setFont(QFont(m_fontFamily, fontSize * 1.3));
            }
            else if (i == 9) {
                s = "W";
                painter.setPen(QPen(m_labelColor));

                painter.setFont(QFont(m_fontFamily, fontSize * 1.3));
            }
            else if (i == 18) {
                s = "S";
                painter.setPen(redPen);

                painter.setFont(QFont(m_fontFamily, fontSize * 1.3));
            }
            else if (i == 27) {
                s = "E";
                painter.setPen(QPen(m_labelColor));

                painter.setFont(QFont(m_fontFamily, fontSize * 1.3));
            }
            else {
                s = QString("%1").arg(i * rotAng);
                painter.setPen(QPen(m_labelColor));

                painter.setFont(QFont(m_fontFamily, fontSize));
            }

            fx1 = 0;
            fy1 = -(size / 2.0 - 2.0 * compassBw);  // start at bezel inner edge
            fx2 = 0;

            if (i % 3 == 0) {
                bool isCardinal = (i == 0 || i == 9 || i == 18 || i == 27);
                int  lineLen = isCardinal ? yawLineLeng * 2 : yawLineLeng * 3 / 2;
                fy2 = fy1 + lineLen;

                if (isCardinal) {
                    // Label sits just inside the bezel; tick starts below the label
                    // so the colored line (blue=N, red=S) doesn't pass through the letter.
                    int    textH     = qRound(fontSize * 1.5) + 2;
                    double tickStart = fy1 + 2 + textH + 2;
                    painter.drawLine(QPointF(fx1, tickStart), QPointF(fx2, fy2));
                    painter.drawText(QRectF(-50, fy1 + 2, 100, textH),
                        Qt::AlignCenter, s);
                } else {
                    painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
                    int textH = fontSize + 2;
                    painter.drawText(QRectF(-50, fy1 + lineLen + 4, 100, textH),
                        Qt::AlignCenter, s);
                }
            }
            else {
                fy2 = fy1 + yawLineLeng / 2;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
            }

            painter.rotate(-rotAng);
        }
        painter.restore();
    }

    // draw S/N arrow
    {
        // Tip must clear the N/S cardinal labels which sit just inside the bezel.
        // labelClear = bezel depth + label text height + gap
        int arrowFontSz = qMax(4, qRound(m_labelSize * m_displayScale));
        int labelClear  = 2 * compassBw + qRound(arrowFontSz * 1.5) + 2 + 8;
        int arrowWidth  = size / 7 * 4 / 5;  // same slim width for both arms
        double  fx1, fy1, fx2, fy2, fx3, fy3;

        painter.setPen(Qt::NoPen);

        // N arm: blue, same slim width as S arm
        fx1 = 0;
        fy1 = -(size / 2.0 - labelClear);
        fx2 = -arrowWidth / 2;
        fy2 = 0;
        fx3 = arrowWidth / 2;
        fy3 = 0;
        painter.setBrush(QBrush(Qt::blue));
        QPointF pointsN[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(pointsN, 3);

        // S arm: red, same width
        fx1 = 0;
        fy1 = size / 2.0 - labelClear;
        fx2 = -arrowWidth / 2;
        fy2 = 0;
        fx3 = arrowWidth / 2;
        fy3 = 0;
        painter.setBrush(QBrush(Qt::red));
        QPointF pointsS[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(pointsS, 3);
    }


    // draw yaw marker (heading indicator) -- black arrowhead, same size as attitude arrowheads
    {
        int    arrowFontSz   = qMax(4, qRound(m_labelSize * m_displayScale));
        int    labelClear    = 2 * compassBw + qRound(arrowFontSz * 1.5) + 2 + 8;
        int    yawMarkerSize = size / 20;  // matches attitude indicator markerSize
        double fx1, fy1, fx2, fy2, fx3, fy3;

        painter.save();
        painter.rotate(-yaw);
        painter.setBrush(QBrush(Qt::black));
        painter.setPen(Qt::NoPen);

        fx1 = 0;
        fy1 = -(size / 2.0 - labelClear);  // tip at same position as N arm tip
        fx2 = fx1 - yawMarkerSize / 2;
        fy2 = fy1 + yawMarkerSize;
        fx3 = fx1 + yawMarkerSize / 2;
        fy3 = fy1 + yawMarkerSize;

        QPointF points[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(points, 3);

        // blue center tick to match the N arm color
        QPen tickPen(Qt::blue, qMax(1, yawMarkerSize / 12));
        painter.setPen(tickPen);
        painter.drawLine(QPointF(fx1, fy1), QPointF(fx1, fy2));

        painter.restore();
    }

    // draw altitude
    {
        int     altFontSize = qMax(6, qRound((m_labelSize + 5) * m_displayScale));
        char    buf[200];
        QString altStr, mslStr;

        if (m_units == Imperial) {
            sprintf(buf, "ALT: %6.1f ft", altitude * 3.28084);
            altStr = buf;
            sprintf(buf, "H: %6.1f ft", msl * 3.28084);
            mslStr = buf;
        } else {
            sprintf(buf, "ALT: %6.1f m", altitude);
            altStr = buf;
            sprintf(buf, "H: %6.1f m", msl);
            mslStr = buf;
        }

        QFont altFont(m_fontFamily, altFontSize);
        painter.setFont(altFont);
        QFontMetrics fm(altFont);
        int textW = qMax(fm.horizontalAdvance(altStr), fm.horizontalAdvance(mslStr));
        int lineH  = fm.lineSpacing();  // safer than height() -- includes leading

        // Horizontal padding scales with font size so the box always has breathing
        // room in fullscreen.  Vertical padding stays fixed.  The drawText rect
        // spans the full box width so AlignCenter works regardless of metrics rounding.
        const int padV = 6;
        const int padH = qMax(16, altFontSize);
        int w  = textW + 2 * padH;
        int h  = 2 * lineH + 3 * padV;  // padV / line / padV / line / padV
        int fx = -w / 2;
        int fy = -h / 2;

        // Dark box with green text -- matches the rest of the avionics UI,
        // avoids white-fill artifacts against the blue compass face
        painter.setPen(Qt::NoPen);
        painter.setBrush(QBrush(QColor(0, 0, 0, 220)));
        painter.drawRoundedRect(fx, fy, w, h, 6, 6);

        painter.setPen(QPen(QColor(0, 220, 0)));
        painter.drawText(QRectF(fx, fy + padV,             w, lineH), Qt::AlignCenter, altStr);
        painter.drawText(QRectF(fx, fy + 2 * padV + lineH, w, lineH), Qt::AlignCenter, mslStr);
    }

    // 3D Metal Bezel ring -- drawn last, on top of all instrument content
    {
        double rO   = size / 2.0;
        int    bw   = qMax(3, (int)rO / 30);
        double rI   = rO - 2.0 * bw;
        double rMid = rO - bw;

        QPainterPath ring;
        ring.addEllipse(QPointF(0, 0), rO, rO);
        ring.addEllipse(QPointF(0, 0), rI, rI);
        ring.setFillRule(Qt::OddEvenFill);

        // Layer 1: Anisotropic metallic sheen -- titanium finish
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

        // Layer 2: Curved-surface sheen
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

void AHRSCompass::keyPressEvent(QKeyEvent* event)
{
    switch (event->key()) {
    case Qt::Key_Left:
        yaw -= 1.0;
        break;
    case Qt::Key_Right:
        yaw += 1.0;
        break;
    case Qt::Key_Down:
        altitude -= 1.0;
        break;
    case Qt::Key_Up:
        altitude += 1.0;
        break;
    case Qt::Key_W:
        msl += 1.0;
        break;
    case Qt::Key_S:
        msl -= 1.0;
        break;

    default:
        QWidget::keyPressEvent(event);
        break;
    }

    update();
}

void AHRSCompass::setInstrFont(const QFont &f) {
    m_fontFamily = f.family();
    if (f.pointSize() > 0) m_labelSize = f.pointSize();
    update();
}

void AHRSCompass::setLabelColor(const QColor &c) { m_labelColor = c; update(); }
void AHRSCompass::setDisplayScale(double scale)  { m_displayScale = scale; update(); }

//From jafrado - added GPS & RTK Control Panel
AHRSInfo::AHRSInfo(QWidget* parent)
{
    m_largePx    = 16;
    m_smallPx    = 12;
    m_fontFamily = "Arial";
    m_dataColor  = QColor(0, 255, 0);
    m_units      = Metric;

    latitude = new QLabel(this);
    latitude->setFrameStyle(QFrame::NoFrame);
    latitude->setStyleSheet(largeStyle());
    //latitude->setGeometry(10,10,80,20);
    latitude->setText("N 000.0000");

    longitude = new QLabel(this);
    longitude->setFrameStyle(QFrame::NoFrame);
    longitude->setStyleSheet(largeStyle());
    longitude->setText("E 000.0000");


    siv = new QLabel(this);
    siv->setFrameStyle(QFrame::NoFrame);
    siv->setStyleSheet(smallStyle());
    siv->setText("0 Sats");

    rtk = new QLabel(this);
    rtk->setFrameStyle(QFrame::NoFrame);
    rtk->setStyleSheet(smallStyle());
    rtk->setText("No RTK");
    //rtk->setFixedWidth(30);

    fix = new QLabel(this);
    fix->setFrameStyle(QFrame::NoFrame);
    fix->setStyleSheet(smallStyle());
    fix->setText("No Fix");
    //fix->setFixedWidth(30);

    pdop = new QLabel(this);
    pdop->setFrameStyle(QFrame::NoFrame);
    pdop->setStyleSheet(smallStyle());
    pdop->setText("0.00");

    QGridLayout* layout = new QGridLayout(this);
    layout->addWidget(latitude, 0, 0);
    layout->addWidget(longitude, 0, 1);
    layout->addWidget(pdop, 0, 2);

    layout->addWidget(siv, 1, 0);
    layout->addWidget(fix, 1, 1);
    layout->addWidget(rtk, 1, 2);

    //layout->addWidget(pdop, 1, 3);


    ecefX = new QLabel(this);
    ecefX->setFrameStyle(QFrame::NoFrame);
    ecefX->setStyleSheet(smallStyle());
    ecefX->setText("Xe 0.00");

    ecefY = new QLabel(this);
    ecefY->setFrameStyle(QFrame::NoFrame);
    ecefY->setStyleSheet(smallStyle());
    ecefY->setText("Ye 0.00");

    ecefZ = new QLabel(this);
    ecefZ->setFrameStyle(QFrame::NoFrame);
    ecefZ->setStyleSheet(smallStyle());
    ecefZ->setText("Ze 0.00");

    layout->addWidget(ecefX, 2, 0);
    layout->addWidget(ecefY, 2, 1);
    layout->addWidget(ecefZ, 2, 2);

    ecefVX = new QLabel(this);
    ecefVX->setFrameStyle(QFrame::NoFrame);
    ecefVX->setStyleSheet(smallStyle());
    ecefVX->setText("Vx 0.00");

    ecefVY = new QLabel(this);
    ecefVY->setFrameStyle(QFrame::NoFrame);
    ecefVY->setStyleSheet(smallStyle());
    ecefVY->setText("Vy 0.00");

    ecefVZ = new QLabel(this);
    ecefVZ->setFrameStyle(QFrame::NoFrame);
    ecefVZ->setStyleSheet(smallStyle());
    ecefVZ->setText("Vz 0.00");

    layout->addWidget(ecefVX, 3, 0);
    layout->addWidget(ecefVY, 3, 1);
    layout->addWidget(ecefVZ, 3, 2);

    // DR estimated position row — lighter text than main GPS labels to indicate estimate
    QString drColor = m_dataColor.lighter(150).name();

    m_drLat = new QLabel(this);
    m_drLat->setFrameStyle(QFrame::NoFrame);
    m_drLat->setStyleSheet(smallStyle(drColor));
    m_drLat->setText("--");

    m_drLon = new QLabel(this);
    m_drLon->setFrameStyle(QFrame::NoFrame);
    m_drLon->setStyleSheet(smallStyle(drColor));
    m_drLon->setText("--");

    m_drAlt = new QLabel(this);
    m_drAlt->setFrameStyle(QFrame::NoFrame);
    m_drAlt->setStyleSheet(smallStyle(drColor));
    m_drAlt->setText("--");

    layout->addWidget(m_drLat, 4, 0);
    layout->addWidget(m_drLon, 4, 1);
    layout->addWidget(m_drAlt, 4, 2);

    // Column 3 stretches to absorb extra horizontal space,
    // keeping columns 0-2 at their natural (content-driven) widths.
    layout->setColumnStretch(3, 1);
}

void AHRSInfo::updateDRPosition(double lat, double lon, double altMslM)
{
    m_drLat->setText(QString("%1 %2").arg(lat >= 0 ? "N" : "S").arg(qAbs(lat), 0, 'f', 5));
    m_drLon->setText(QString("%1 %2").arg(lon >= 0 ? "E" : "W").arg(qAbs(lon), 0, 'f', 5));
    if (m_units == Imperial)
        m_drAlt->setText(QString("%1 ft").arg(qRound(altMslM * 3.28084)));
    else
        m_drAlt->setText(QString("%1 m").arg(altMslM, 0, 'f', 1));
}

void AHRSInfo::setUnits(Units u)
{
    m_units = u;
}

void AHRSInfo::updatePosition(double lat, double lon)
{
    QString ns = "N";
    QString ew = "E";
    if (lat < 0)
        ns = "S";
    if (lon < 0)
        ew = "W";
    latitude->setText(QString("%1 %2").arg(lat < 0 ? 'S' : 'N').arg(qFabs(lat), 5, 'f', 4, '0'));
    longitude->setText(QString("%1 %2").arg(lon < 0 ? 'W' : 'E').arg(qFabs(lon), 5, 'f', 4, '0'));
}

void AHRSInfo::updatePDOP(double prec)
{
    prec /= 10; /* cm */
    pdop->setText(QString::number(prec, 'f', 2) + " cm");
    if (prec  > 200.0)
        pdop->setStyleSheet(smallStyle("rgb(255, 0, 0)"));
    else if ( (prec < 200.0) && (prec > 20.0))
        pdop->setStyleSheet(smallStyle("rgb(255, 215, 0)"));
    else if (prec < 20.0)
        pdop->setStyleSheet(smallStyle());
    else
        pdop->setStyleSheet(smallStyle("rgb(255, 0, 0)"));


}

void AHRSInfo::updateStatus(int inview, int fixType, int rtkType)
{
    QString fixInfo;
    QString rtkInfo;

    siv->setText(QString::number(inview) + " Sats");

    //0 = none, 1 = Dead Reckoning, 2 = 2D, 3 = 3D, 4 = GNSS + Dead Reckoning
    switch (fixType) {
    default:
    case 0:
        fixInfo = "No Fix";
        break;
    case 1:
        fixInfo = "DR Fix";
        break;
    case 2:
        fixInfo = "2D Fix";
        break;
    case 3:
        fixInfo = "3D Fix";
        break;
    case 4:
        fixInfo = "GNSS DR";
        break;

    }
    fix->setText(fixInfo);

    if (fixType < 3) {
        pdop->setStyleSheet(smallStyle("rgb(255, 0, 0)"));
    }
    else {
        pdop->setStyleSheet(smallStyle());
    }

    

    //0 = na, 1 = High Precision Float Fix, 2 = High precision fix
    switch (rtkType) {
    default:
    case 0:
        rtkInfo = "N/A";
        break;
    case 1:
        rtkInfo = "RTK Float";
        break;
    case 2:
        rtkInfo = "RTK Fix";
        break;
    }
    rtk->setText(rtkInfo);
    if (rtkType == 0) {
        pdop->setStyleSheet(smallStyle("rgb(255, 0, 0)"));
    }
    else {
        pdop->setStyleSheet(smallStyle());
    }

    
}

QString AHRSInfo::largeStyle(const QString &color) const
{
    QString c = color.isEmpty() ? m_dataColor.name() : color;
    return QString("QLabel{font-size: %1px; font-family: %2; color: %3; background-color: rgb(0,0,0);}")
        .arg(m_largePx).arg(m_fontFamily).arg(c);
}

QString AHRSInfo::smallStyle(const QString &color) const
{
    QString c = color.isEmpty() ? m_dataColor.name() : color;
    return QString("QLabel{font-size: %1px; font-family: %2; color: %3; background-color: rgb(0,0,0);}")
        .arg(m_smallPx).arg(m_fontFamily).arg(c);
}

void AHRSInfo::setFontScale(double scale)
{
    m_largePx = qRound(16 * scale);
    m_smallPx = qRound(12 * scale);
    applyStyles();
}

void AHRSInfo::applyStyles()
{
    latitude->setStyleSheet(largeStyle());
    longitude->setStyleSheet(largeStyle());
    siv->setStyleSheet(smallStyle());
    fix->setStyleSheet(smallStyle());
    rtk->setStyleSheet(smallStyle());
    pdop->setStyleSheet(smallStyle());
    ecefX->setStyleSheet(smallStyle());
    ecefY->setStyleSheet(smallStyle());
    ecefZ->setStyleSheet(smallStyle());
    ecefVX->setStyleSheet(smallStyle());
    ecefVY->setStyleSheet(smallStyle());
    ecefVZ->setStyleSheet(smallStyle());
    QString drColor = m_dataColor.lighter(150).name();
    m_drLat->setStyleSheet(smallStyle(drColor));
    m_drLon->setStyleSheet(smallStyle(drColor));
    m_drAlt->setStyleSheet(smallStyle(drColor));
}

void AHRSInfo::setFontFamily(const QString &fam) { m_fontFamily = fam; applyStyles(); }
void AHRSInfo::setTextColor(const QColor &c)     { m_dataColor  = c;   applyStyles(); }

AHRSInfo::~AHRSInfo()
{}

