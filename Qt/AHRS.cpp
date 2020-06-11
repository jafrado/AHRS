/***
 * AHRS Receiver - Receive AHRS messages from IMU FIrmware via QT Serial Port 
 * AHRS Widget  - Instantiate AHRS Sub-Widgets (Attitude Indicator, Compass, Info, etc)
 * AHRS MainWindow - presentation window for Widget/App
 */

#include "AHRS.h"
#include <chrono>


AHRSReceiver::AHRSReceiver(QString devName)
    :lines(0), deviceName(devName), done(true)
{
    //Adjust for your hardware/board
    //These offsets will correct for any mounting peculiarities
    ahrsData.rollPolarity = -1; //Switch to cockpit view (left stick, port roll)
}
AHRSReceiver::~AHRSReceiver()
{
}

void AHRSReceiver::begin()
{
    QThreadPool::globalInstance()->start(this);
}
void AHRSReceiver::end()
{
    if (!done) {
        done = true;
        QThreadPool::globalInstance()->waitForDone();
        done = true;

    }
}

void AHRSReceiver::restart()
{
    end();
    begin();
}
AHRSData& AHRSReceiver::getData()
{
    return ahrsData;
}

bool AHRSReceiver::isConnected()
{
    return !done;
}

bool AHRSReceiver::validPosition()
{
    return ahrsData.fixType >= 2; //should be 3
}

void AHRSReceiver::run()
{
    QSerialPort serialPort;
    QString fixInfo, rtkInfo;
    AHRSData current;


    try {
        qDebug() << "AHRS Data receiver on[" << deviceName << "]:" << QThread::currentThread();
        serialPort.setPortName(deviceName);

        const int serialPortBaudRate = QSerialPort::Baud115200;
        serialPort.setBaudRate(serialPortBaudRate);

        if (!serialPort.open(QIODevice::ReadWrite)) {
            qDebug() << QObject::tr("Failed to open port %1, error: %2").arg(deviceName).arg(serialPort.error()) << endl;
            done = true;
            return;
        }
        //Arduino: set DTR to take board out of reset
        serialPort.setDataTerminalReady(true);

        //wait for one second before reading ...
        serialPort.waitForReadyRead(1000);
        //serialPort.write("\r\n");

        QByteArray readData(1024, '0');
        QStringList lineTokens;
        QString line;
        done = false;

        while (!done) {


            //mark start time
            std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
            //readData.clear();
            //Synch to $ char ..
            readData = serialPort.readAll();
            while (serialPort.waitForReadyRead(10) && !readData.contains('$')) {
                serialPort.readAll();
            }


            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

            //continue to read entire line
            while (serialPort.waitForReadyRead(100) && !readData.contains('\r')) {
                readData += serialPort.readAll();
            }

            if (readData.size() < 15) /* we need at least $INS ... */
                continue;

            //mark end time
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            double syncTime = std::chrono::duration_cast<std::chrono::microseconds>(begin - t0).count();
            double readLineTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            //qDebug() << "line-read time = " << readLineTime<< "[us], synch-time:" << syncTime ;

            if (serialPort.error() == QSerialPort::ReadError) {
                qDebug() << QObject::tr("Failed to read from port %1, error: %2").arg(deviceName).arg(serialPort.errorString()) << endl;
                done = true;
                break;
            }
            else if (serialPort.error() == QSerialPort::TimeoutError && readData.isEmpty()) {
                std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
                double dataEmptyTime = std::chrono::duration_cast<std::chrono::microseconds>(end2 - end).count();
                // qDebug() << "data-empty time = " << dataEmptyTime<< "[us]";
                qDebug() << QObject::tr("No data was currently available for reading from port %1").arg(deviceName) << "st:" << syncTime << ",et: " << dataEmptyTime << ",rt:" << readLineTime << ",bytes:" << readData.size() << endl;
                serialPort.waitForReadyRead(100);
                //  serialPort.write("\r\n");
            }

            //lineTokens.clear();
            line = QString(readData);
            //qDebug() <<"[" << readData << "]";
           // qDebug() << line;

            lineTokens = line.split(',');

            //we need to at least update attitude atomically, so we check for at least the first 3 tokens and format ...
            if (lineTokens.size() >= 4) {
                ++lines;
                //qDebug() << line;
                 //Custom INS (Inertial Nav System) - output baked from a processor downstream hanging off the UART, see Arduino firmware reference
                if (lineTokens[0].startsWith("$INS")) {
                    //qDebug() << "Tokens (" << lineTokens.size() << "):" << lineTokens;

                    //AHRS library already uses Quat internally for Euler angles to prevent gimbal lock
                    if ((lineTokens.size() > 1) && (lineTokens[1].size() > 0)) {
                        ahrsData.yaw = lineTokens[1].toDouble();
                        ahrsData.yaw = ahrsData.yaw * ahrsData.yawPolarity + ahrsData.yawOffset;
                    }
                    if ((lineTokens.size() > 2) && (lineTokens[2].size() > 0)) {
                        ahrsData.pitch = lineTokens[2].toDouble();
                        ahrsData.pitch = ahrsData.pitch * ahrsData.pitchPolarity + ahrsData.pitchOffset;
                    }
                    if ((lineTokens.size() > 3) && (lineTokens[3].size() > 0)) {
                        ahrsData.roll = lineTokens[3].toDouble();
                        ahrsData.roll = ahrsData.roll * ahrsData.rollPolarity + ahrsData.rollOffset;
                    }
                    //qDebug() << "h1="   << yaw << " p1=" << pitch << " r1=" << roll;
                    if ((current.yaw != ahrsData.yaw) || (current.pitch != ahrsData.pitch) || (current.roll != ahrsData.roll)) {
                        emit poseChanged(ahrsData.yaw, ahrsData.pitch, ahrsData.roll);
                        //qDebug() << "h1=" << ahrsData.yaw << " p1=" << ahrsData.pitch << " r1=" << ahrsData.roll;
                    }
                    //gps data follows
                    //WGS84 coords
                    if ((lineTokens.size() > 5) && (lineTokens[4].size() > 0) && (lineTokens[5].size() > 0)) {
                        ahrsData.lat = lineTokens[4].toDouble() * 1E-07;
                        ahrsData.lon = lineTokens[5].toDouble() * 1E-07;
                    }
                    //altitude in mm (GPS altitude, or height above ellipsoid)
                    if ((lineTokens.size() > 6) && (lineTokens[6].size() > 0)) {
                        ahrsData.altitude = lineTokens[6].toDouble(); //In MM
                        ahrsData.altitude /= 1000;
                    }
                    //MSL altitude in mm
                    if ((lineTokens.size() > 7) && (lineTokens[7].size() > 0)) {
                        ahrsData.altitudeMSL = lineTokens[7].toDouble(); //In MM
                        ahrsData.altitudeMSL /= 1000;
                    }
                    //position with altitude
                    if ((current.lat != ahrsData.lat) ||
                        (current.lon != ahrsData.lon) ||
                        (current.altitude != ahrsData.altitude)) {
                        emit positionChanged(ahrsData.lat, ahrsData.lon, ahrsData.altitude);
                        //qDebug() << "lat=" << ahrsData.lat << " lon=" << ahrsData.lon << " alt=" << ahrsData.altitude;
                    }
                    //altitude including MSL
                    if ((current.altitude != ahrsData.altitude) ||
                        (current.altitudeMSL != ahrsData.altitudeMSL)) {
                        emit altitudeChanged(ahrsData.altitude, ahrsData.altitudeMSL);
                        //qDebug() << "alt=" << ahrsData.altitude << " msl=" << ahrsData.altitudeMSL;
                    }
                    //satellites in view
                    if ((lineTokens.size() > 8) && (lineTokens[8].size() > 0)) {
                        ahrsData.siv = lineTokens[8].toInt();
                    }
                    //0 = none, 1 = Dead Reckoning, 2 = 2D, 3 = 3D, 4 = GNSS + Dead Reckoning
                    if ((lineTokens.size() > 9) && (lineTokens[9].size() > 0)) {
                        ahrsData.fixType = lineTokens[9].toInt();
                    }
                    //0 = na, 1 = High Precision Float Fix, 2 = High precision fix
                    if ((lineTokens.size() > 10) && (lineTokens[10].size() > 0)) {
                        ahrsData.rtkType = lineTokens[10].toInt();
                    }
                    //uBlox lock / status notification
                    if ((current.siv != ahrsData.siv) ||
                        (current.fixType != ahrsData.fixType) ||
                        (current.rtkType != ahrsData.rtkType)) {
                        emit statusChanged(ahrsData.siv, ahrsData.fixType, ahrsData.rtkType);
                        //qDebug() << "siv=" << ahrsData.siv << " ft=" << ahrsData.fixType << " rt=" << ahrsData.rtkType;
                    }
                    //compass bearing
                    if ((lineTokens.size() > 11) && (lineTokens[11].size() > 0)) {
                        ahrsData.heading = lineTokens[11].toDouble();
                    }
                    if (current.heading != ahrsData.heading) {
                        emit headingChanged(ahrsData.heading);
                        //qDebug() << "bng=" << ahrsData.heading;
                    }
                    //ground speed
                    if ((lineTokens.size() > 12) && (lineTokens[12].size() > 0)) {
                        ahrsData.ground_speed = lineTokens[12].toDouble() / 1000;
                    }
                    if (current.ground_speed != ahrsData.ground_speed) {
                        emit groundSpeedChanged(ahrsData.ground_speed);
                        //qDebug() << "gs=" << ahrsData.ground_speed;
                    }
                    //position dilution of precision
                    if ((lineTokens.size() > 13) && (lineTokens[13].size() > 0)) {
                        ahrsData.pdop = lineTokens[13].toDouble();
                        //qDebug() << "pdop:" << ahrsData.pdop;
                    }
                    if (current.pdop != ahrsData.pdop) {
                        emit precisionChanged(ahrsData.pdop);
                        //qDebug() << "pdop=" << ahrsData.pdop;
                    }
                    //ECEF X - Earth Radius = 6.37E06 m (~10^6)
                    if ((lineTokens.size() > 14) && (lineTokens[14].size() > 0)) {
                        //ahrsData.x = lineTokens[14].toDouble() / 1.0E7;
                        ahrsData.x = lineTokens[14].toDouble();
                    }
                    //ECEF Y
                    if ((lineTokens.size() > 15) && (lineTokens[15].size() > 0)) {
                        //ahrsData.y = lineTokens[15].toDouble() / 1.0E7;
                        ahrsData.y = lineTokens[15].toDouble();
                    }
                    //ECEF Z
                    if ((lineTokens.size() > 16) && (lineTokens[16].size() > 0)) {
                        //ahrsData.z = lineTokens[16].toDouble() / 1.0E7;
                        ahrsData.z = lineTokens[16].toDouble();
                    }
                    if ((current.x != ahrsData.x) ||
                        (current.y != ahrsData.y) ||
                        (current.z != ahrsData.z)) {
                        emit positionECEFChanged(ahrsData.x, ahrsData.y, ahrsData.z);
                        qDebug() << "x=" << ahrsData.x << " y=" << ahrsData.y << " z=" << ahrsData.z;
                    }
                    //ECEF VX
                    if ((lineTokens.size() > 17) && (lineTokens[17].size() > 0)) {
                        ahrsData.vx = lineTokens[17].toDouble();
                    }
                    //ECEF VY
                    if ((lineTokens.size() > 18) && (lineTokens[18].size() > 0)) {
                        ahrsData.vy = lineTokens[18].toDouble();
                    }
                    //ECEF VZ
                    if ((lineTokens.size() > 19) && (lineTokens[19].size() > 0)) {
                        ahrsData.vz = lineTokens[19].toDouble();
                    }
                    if ((current.vx != ahrsData.vx) ||
                        (current.vy != ahrsData.vy) ||
                        (current.vz != ahrsData.vz)) {
                        emit speedChanged(ahrsData.vx, ahrsData.vy, ahrsData.vz);
                        //qDebug() << "vx=" << ahrsData.vx << " vy=" << ahrsData.vy << " vz=" << ahrsData.vz;
                    }
                    ahrsData.seqno = lines;
                }
                //NMEA GPS receieve - crack lat/lon, altitude
                else if (lineTokens[0].startsWith("$GP")) {
                    //@@TODO
                }
                else {
                    //some new type of droid here
                }
                current = ahrsData;

                /* Do one update of HUD per status line ... */
                emit redrawDisplay();
            }
        }

    }
    catch (QException e) {
        qDebug() << e.what();
    }
    serialPort.close();
    qDebug() << QThread::currentThread() << " done";
    done = true;
}



AHRSWidget::AHRSWidget(QWidget* parent)
    : QWidget(parent),
    deviceName("none")
{
    QVBoxLayout* vl = new QVBoxLayout(this);
    deviceLabel = new QLabel(this);
    setLayout(vl);
    setFocusPolicy(Qt::NoFocus);
    attitude = new AHRSAttitudeIndicator(this);
    heading = new AHRSCompass(this);
    info = new AHRSInfo(this);
    //vl->addWidget(info,  0, Qt::AlignTop|Qt::AlignHCenter);
    vl->addWidget(attitude, 0, Qt::AlignTop | Qt::AlignHCenter);
    vl->addWidget(heading, 0, Qt::AlignTop | Qt::AlignHCenter);
    vl->addWidget(info, 0, Qt::AlignTop | Qt::AlignHCenter);
    vl->addWidget(deviceLabel, 0, Qt::AlignBottom | Qt::AlignHCenter);
    deviceLabel->setText(deviceName);
    vl->setMargin(0);
    vl->setSpacing(0);
}

void AHRSWidget::updatePose(double yaw, double pitch, double roll)
{
    heading->setYaw(yaw);
    attitude->setPitch(pitch);
    attitude->setRoll(roll);
    //qDebug() << "h1=" << yaw << " p1=" << pitch << " r1=" << roll;
}

void AHRSWidget::updatePosition(double lat, double lon, double altitude)
{
    info->updatePosition(lat, lon);
    heading->setH(altitude);
}

void AHRSWidget::updateECEFPosition(double ecefX, double ecefY, double ecefZ)
{
    info->ecefX->setText("Xe:" + QString::number(ecefX, 'f', 2));
    info->ecefY->setText("Ye:" + QString::number(ecefY, 'f', 2));
    info->ecefZ->setText("Ze:" + QString::number(ecefZ, 'f', 2));
}

void AHRSWidget::updateAltitude(double altitude, double altitudeMSL)
{
    heading->setH(altitude);
    heading->setAlt(altitudeMSL);
}

void AHRSWidget::updateGroundSpeed(double ground_speed)
{}

void AHRSWidget::updateSpeed(double vx, double vy, double vz)
{
    info->ecefVX->setText("Vx:" + QString::number(vx, 'f', 2));
    info->ecefVY->setText("Vy:" + QString::number(vy, 'f', 2));
    info->ecefVZ->setText("Vz:" + QString::number(vz, 'f', 2));
}

void AHRSWidget::updateHeading(double bearing)
{}
void AHRSWidget::updatePrecision(double precision)
{
    info->updatePDOP(precision);
}

void AHRSWidget::updateStatus(int siv, int fixType, int rtkType)
{
    info->updateStatus(siv, fixType, rtkType);
}


//for debug and testing
void AHRSWidget::keyPressEvent(QKeyEvent* event)
{
    int     key;
    double  v;
    key = event->key();
    if (key == Qt::Key_Up) {
        v = attitude->getPitch();
        attitude->setPitch(v + 1.0);
    }
    else if (key == Qt::Key_Down) {
        v = attitude->getPitch();
        attitude->setPitch(v - 1.0);
    }
    else if (key == Qt::Key_Left) {
        v = attitude->getRoll();
        attitude->setRoll(v - 1.0);
    }
    else if (key == Qt::Key_Right) {
        v = attitude->getRoll();
        attitude->setRoll(v + 1.0);
    }
    else if (key == Qt::Key_A) {
        v = heading->getYaw();
        heading->setYaw(v + 1.0);
    }
    else if (key == Qt::Key_D) {
        v = heading->getYaw();
        heading->setYaw(v - 1.0);
    }
    else if (key == Qt::Key_W) {
        v = heading->getAlt();
        heading->setAlt(v + 1.0);
    }
    else if (key == Qt::Key_S) {
        v = heading->getAlt();
        heading->setAlt(v - 1.0);
    }
    else if (key == Qt::Key_J) {
        v = heading->getH();
        heading->setH(v + 1.0);
    }
    else if (key == Qt::Key_K) {
        v = heading->getH();
        heading->setH(v - 1.0);
    }
}

void AHRSWidget::updateHUD()
{
    /* Do one update of HUD per status line ... */
    attitude->update();
    heading->update();
}

//setup widget for asynch update
void AHRSWidget::setReceiver(AHRSReceiver* receiver)
{
    this->receiver = receiver;
    connect(receiver, SIGNAL(poseChanged(double, double, double)), this, SLOT(updatePose(double, double, double)));
    connect(receiver, SIGNAL(positionChanged(double, double, double)), this, SLOT(updatePosition(double, double, double)));
    connect(receiver, SIGNAL(positionECEFChanged(double, double, double)), this, SLOT(updateECEFPosition(double, double, double)));
    connect(receiver, SIGNAL(altitudeChanged(double, double)), this, SLOT(updateAltitude(double, double)));
    connect(receiver, SIGNAL(speedChanged(double, double, double)), this, SLOT(updateSpeed(double, double, double)));
    connect(receiver, SIGNAL(groundSpeedChanged(double)), this, SLOT(updateGroundSpeed(double)));
    connect(receiver, SIGNAL(precisionChanged(double)), this, SLOT(updatePrecision(double)));
    connect(receiver, SIGNAL(statusChanged(int, int, int)), this, SLOT(updateStatus(int, int, int)));
    connect(receiver, SIGNAL(headingChanged(double)), this, SLOT(updateHeading(double)));
    connect(receiver, SIGNAL(redrawDisplay()), this, SLOT(updateHUD()));
}

