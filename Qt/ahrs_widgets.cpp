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
    : QWidget(parent)
{
    //QTimer *timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    //timer->start(500);

    //connect(this, SIGNAL(widgetReplot(void)), this, SLOT(widgetReplot_slot(void)));
    offset = 2;
    size = 180;
    setMinimumSize(180, 180);
    setMaximumSize(600, 600);
    resize(size, size);
    setFocusPolicy(Qt::NoFocus);
    roll = 0.0;
    pitch = 0.0;

}

AHRSAttitudeIndicator::~AHRSAttitudeIndicator()
{}

void AHRSAttitudeIndicator::resizeEvent(QResizeEvent* event)
{
    size = qMin(width(), height()) - 2 * offset;
    QWidget::resizeEvent(event);
}

void AHRSAttitudeIndicator::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
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


    // set mask
    QRegion maskRegion(-size / 2, -size / 2, size, size, QRegion::Ellipse);
    painter.setClipRegion(maskRegion);


    // draw pitch lines & marker
    {
        int x, y, x1, y1;
        int textWidth;
        double p, r;
        int ll = size / 8, l;

        int     fontSize = 8;
        QString s;

        pitchPen.setWidth(2);
        painter.setFont(QFont("Arial", fontSize));


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
                painter.setPen(QPen(Qt::white));

                x1 = -x - 2 - textWidth;
                y1 = y - fontSize / 2 - 1;
                painter.drawText(QRectF(x1, y1, textWidth, fontSize + 2),
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
    }

    // draw roll degree lines
    {
        int     nRollLines = 36;
        float   rotAng = 360.0 / nRollLines;
        int     rollLineLeng = size / 25;
        double  fx1, fy1, fx2, fy2;
        int     fontSize = 8;
        QString s;

        blackPen.setWidth(1);
        painter.setPen(blackPen);
        painter.setFont(QFont("Arial", fontSize));

        for (int i = 0; i < nRollLines; i++) {
            if (i < nRollLines / 2)
                s = QString("%1").arg(-i * rotAng);
            else
                s = QString("%1").arg(360 - i * rotAng);

            fx1 = 0;
            fy1 = -size / 2 + offset;
            fx2 = 0;

            if (i % 3 == 0) {
                fy2 = fy1 + rollLineLeng;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));

                fy2 = fy1 + rollLineLeng + 2;
                painter.drawText(QRectF(-50, fy2, 100, fontSize + 2),
                    Qt::AlignCenter, s);
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
        int     rollMarkerSize = size / 25;
        double  fx1, fy1, fx2, fy2, fx3, fy3;

        painter.rotate(-roll);
        painter.setBrush(QBrush(Qt::black));

        fx1 = 0;
        fy1 = -size / 2 + offset;
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

//from bushuhui
AHRSCompass::AHRSCompass(QWidget* parent)
    : QWidget(parent)
{
    //QTimer *timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    //timer->start(500);


    //connect(this, SIGNAL(canvasReplot(void)), this, SLOT(canvasReplot_slot(void)));

    sizeMin = 180;
    sizeMax = 600;
    offset = 2;
    size = sizeMin - 2 * offset;

    setMinimumSize(sizeMin, sizeMin);
    setMaximumSize(sizeMax, sizeMax);
    resize(sizeMin, sizeMin);

    setFocusPolicy(Qt::NoFocus);

    yaw = 0.0;
    altitude = 0.0;
    msl = 0.0;
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
}

void AHRSCompass::paintEvent(QPaintEvent*)
{

    QPainter painter(this);

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


    // draw yaw lines
    {
        int     nyawLines = 36;
        float   rotAng = 360.0 / nyawLines;
        int     yawLineLeng = size / 25;
        double  fx1, fy1, fx2, fy2;
        int     fontSize = 8;
        QString s;

        blackPen.setWidth(1);
        painter.setPen(blackPen);

        for (int i = 0; i < nyawLines; i++) {

            if (i == 0) {
                s = "N";
                painter.setPen(bluePen);

                painter.setFont(QFont("Arial", fontSize * 1.3));
            }
            else if (i == 9) {
                s = "W";
                painter.setPen(blackPen);

                painter.setFont(QFont("Arial", fontSize * 1.3));
            }
            else if (i == 18) {
                s = "S";
                painter.setPen(redPen);

                painter.setFont(QFont("Arial", fontSize * 1.3));
            }
            else if (i == 27) {
                s = "E";
                painter.setPen(blackPen);

                painter.setFont(QFont("Arial", fontSize * 1.3));
            }
            else {
                s = QString("%1").arg(i * rotAng);
                painter.setPen(blackPen);

                painter.setFont(QFont("Arial", fontSize));
            }

            fx1 = 0;
            fy1 = -size / 2 + offset;
            fx2 = 0;

            if (i % 3 == 0) {
                fy2 = fy1 + yawLineLeng;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));

                fy2 = fy1 + yawLineLeng + 4;
                painter.drawText(QRectF(-50, fy2, 100, fontSize + 2),
                    Qt::AlignCenter, s);
            }
            else {
                fy2 = fy1 + yawLineLeng / 2;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
            }

            painter.rotate(-rotAng);
        }
    }

    // draw S/N arrow
    {
        int     arrowWidth = size / 5;
        double  fx1, fy1, fx2, fy2, fx3, fy3;

        fx1 = 0;
        fy1 = -size / 2 + offset + size / 25 + 15;
        fx2 = -arrowWidth / 2;
        fy2 = 0;
        fx3 = arrowWidth / 2;
        fy3 = 0;

        painter.setPen(Qt::NoPen);

        painter.setBrush(QBrush(Qt::blue));
        QPointF pointsN[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(pointsN, 3);


        fx1 = 0;
        fy1 = size / 2 - offset - size / 25 - 15;
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


    // draw yaw marker
    {
        int     yawMarkerSize = size / 12;
        double  fx1, fy1, fx2, fy2, fx3, fy3;

        painter.rotate(-yaw);
        painter.setBrush(QBrush(QColor(0xFF, 0x00, 0x00, 0xE0)));

        fx1 = 0;
        fy1 = -size / 2 + offset;
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

        painter.rotate(yaw);
    }

    // draw altitude
    {
        int     altFontSize = 13;
        int     fx, fy, w, h;
        QString s;
        char    buf[200];

        w = 130;
        h = 2 * (altFontSize + 8);
        fx = -w / 2;
        fy = -h / 2;

        blackPen.setWidth(2);
        painter.setPen(blackPen);
        painter.setBrush(QBrush(Qt::white));
        painter.setFont(QFont("Arial", altFontSize));

        painter.drawRoundedRect(fx, fy, w, h, 6, 6);

        painter.setPen(bluePen);
        sprintf(buf, "ALT: %6.1f m", altitude);
        s = buf;
        painter.drawText(QRectF(fx, fy + 2, w, h / 2), Qt::AlignCenter, s);

        sprintf(buf, "H: %6.1f m", msl);
        s = buf;
        painter.drawText(QRectF(fx, fy + h / 2, w, h / 2), Qt::AlignCenter, s);
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

//From jafrado - added GPS & RTK Control Panel
AHRSInfo::AHRSInfo(QWidget* parent)
{
    latitude = new QLabel(this);
    latitude->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    latitude->setStyleSheet("QLabel{font-size: 16px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    //latitude->setGeometry(10,10,80,20);
    latitude->setText("N 000.0000");

    longitude = new QLabel(this);
    longitude->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    longitude->setStyleSheet("QLabel{font-size: 16px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    longitude->setText("E 000.0000");


    siv = new QLabel(this);
    siv->setFixedWidth(70);
    siv->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    siv->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    siv->setText("0 Sats");

    rtk = new QLabel(this);
    rtk->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    rtk->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    rtk->setText("No RTK");
    //rtk->setFixedWidth(30);

    fix = new QLabel(this);
    fix->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    fix->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    fix->setText("No Fix");
    //fix->setFixedWidth(30);

    pdop = new QLabel(this);
    pdop->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
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
    ecefX->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ecefX->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    ecefX->setText("Xe 0.00");

    ecefY = new QLabel(this);
    ecefY->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ecefY->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    ecefY->setText("Ye 0.00");

    ecefZ = new QLabel(this);
    ecefZ->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ecefZ->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    ecefZ->setText("Ze 0.00");

    layout->addWidget(ecefX, 2, 0);
    layout->addWidget(ecefY, 2, 1);
    layout->addWidget(ecefZ, 2, 2);

    ecefVX = new QLabel(this);
    ecefVX->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ecefVX->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    ecefVX->setText("Vx 0.00");

    ecefVY = new QLabel(this);
    ecefVY->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ecefVY->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    ecefVY->setText("Vy 0.00");

    ecefVZ = new QLabel(this);
    ecefVZ->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ecefVZ->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    ecefVZ->setText("Vz 0.00");

    layout->addWidget(ecefVX, 3, 0);
    layout->addWidget(ecefVY, 3, 1);
    layout->addWidget(ecefVZ, 3, 2);


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
        pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(255, 0, 0);background-color: rgb(0,0,0);}");
    else if ( (prec < 200.0) && (prec > 20.0))
        pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(255, 215, 0);background-color: rgb(0,0,0);}");
    else if (prec < 20.0)
        pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    else
        pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(255, 0, 0);background-color: rgb(0,0,0);}");


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
        pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(255, 0, 0);background-color: rgb(0,0,0);}");
    }
    else {
        pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
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
    if (rtkInfo < 1) {
        pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(255, 0, 0);background-color: rgb(0,0,0);}");
    }
    else {
        pdop->setStyleSheet("QLabel{font-size: 12px;font-family: Arial;color: rgb(0, 255, 0);background-color: rgb(0,0,0);}");
    }

    
}

AHRSInfo::~AHRSInfo()
{}

