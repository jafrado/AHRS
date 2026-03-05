/***
 * AHRS Receiver - Receive AHRS messages from IMU Firmware via serial port or TCP
 * AHRS Widget  - Instantiate AHRS Sub-Widgets (Attitude Indicator, Compass, Info, etc)
 * AHRS MainWindow - presentation window for Widget/App
 */

#include "AHRS.h"
#include <chrono>
#include <cmath>
#include <QSettings>
#include <QCoreApplication>

// AHRSReceiver

AHRSReceiver::AHRSReceiver(QString devName)
    : lines(0), deviceName(devName), mode(Serial), tcpHost(""), tcpPort(0), done(true)
{
    ahrsData.rollPolarity = -1; //Switch to cockpit view (left stick, port roll)
}

AHRSReceiver::AHRSReceiver(QString host, quint16 port)
    : lines(0), deviceName(""), mode(TCP), tcpHost(host), tcpPort(port), done(true)
{
    ahrsData.rollPolarity = -1;
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
    return ahrsData.fixType >= 2;
}

// parseLine: shared by serial and TCP paths

void AHRSReceiver::parseLine(const QByteArray &readData, AHRSData &current)
{
    QString line = QString(readData);
    QStringList lineTokens = line.split(',');

    if (lineTokens.size() < 4)
        return;

    ++lines;
    if (lineTokens[0].startsWith("$INS")) {
        // --- Parse all fields first ---
        if (lineTokens.size() > 1  && !lineTokens[1].isEmpty())  {
            ahrsData.yaw   = lineTokens[1].toDouble() * ahrsData.yawPolarity   + ahrsData.yawOffset;
        }
        if (lineTokens.size() > 2  && !lineTokens[2].isEmpty())  {
            ahrsData.pitch = lineTokens[2].toDouble() * ahrsData.pitchPolarity + ahrsData.pitchOffset;
        }
        if (lineTokens.size() > 3  && !lineTokens[3].isEmpty())  {
            ahrsData.roll  = lineTokens[3].toDouble() * ahrsData.rollPolarity  + ahrsData.rollOffset;
        }
        if (lineTokens.size() > 5  && !lineTokens[4].isEmpty() && !lineTokens[5].isEmpty()) {
            ahrsData.lat = lineTokens[4].toDouble() * 1E-07;
            ahrsData.lon = lineTokens[5].toDouble() * 1E-07;
        }
        if (lineTokens.size() > 6  && !lineTokens[6].isEmpty())  {
            ahrsData.altitude    = lineTokens[6].toDouble() / 1000.0;
        }
        if (lineTokens.size() > 7  && !lineTokens[7].isEmpty())  {
            ahrsData.altitudeMSL = lineTokens[7].toDouble() / 1000.0;
        }
        if (lineTokens.size() > 8  && !lineTokens[8].isEmpty())  {
            ahrsData.siv     = lineTokens[8].toInt();
        }
        if (lineTokens.size() > 9  && !lineTokens[9].isEmpty())  {
            ahrsData.fixType = lineTokens[9].toInt();
        }
        if (lineTokens.size() > 10 && !lineTokens[10].isEmpty()) {
            ahrsData.rtkType = lineTokens[10].toInt();
        }
        if (lineTokens.size() > 11 && !lineTokens[11].isEmpty()) {
            ahrsData.heading      = lineTokens[11].toDouble();
        }
        if (lineTokens.size() > 12 && !lineTokens[12].isEmpty()) {
            ahrsData.ground_speed = lineTokens[12].toDouble() / 1000.0;
        }
        if (lineTokens.size() > 13 && !lineTokens[13].isEmpty()) {
            ahrsData.pdop = lineTokens[13].toDouble();
        }
        if (lineTokens.size() > 14 && !lineTokens[14].isEmpty()) { ahrsData.x = lineTokens[14].toDouble(); }
        if (lineTokens.size() > 15 && !lineTokens[15].isEmpty()) { ahrsData.y = lineTokens[15].toDouble(); }
        if (lineTokens.size() > 16 && !lineTokens[16].isEmpty()) { ahrsData.z = lineTokens[16].toDouble(); }
        if (lineTokens.size() > 17 && !lineTokens[17].isEmpty()) { ahrsData.vx = lineTokens[17].toDouble(); }
        if (lineTokens.size() > 18 && !lineTokens[18].isEmpty()) { ahrsData.vy = lineTokens[18].toDouble(); }
        if (lineTokens.size() > 19 && !lineTokens[19].isEmpty()) { ahrsData.vz = lineTokens[19].toDouble(); }
        ahrsData.seqno = lines;

        // --- GPS-aided dead reckoning ---
        // Seed from GPS when fix is valid; always integrate using NED velocity.
        if (ahrsData.fixType >= 2)
            m_gpsInt.seed(ahrsData.lat, ahrsData.lon, ahrsData.altitudeMSL);

        double dt = 0.1;
        if (m_intTimerStarted)
            dt = qBound(0.01, m_intTimer.elapsed() / 1000.0, 1.0);
        m_intTimer.restart();
        m_intTimerStarted = true;

        m_gpsInt.integrate(ahrsData.vx, ahrsData.vy, ahrsData.altitudeMSL, dt);
        if (m_gpsInt.isSeeded())
            emit estimatedPositionChanged(m_gpsInt.latDeg(), m_gpsInt.lonDeg(), m_gpsInt.altMslM());

        // --- Emit all signals unconditionally ---
        // Change-detection guards on float fields caused -Wfloat-equal warnings
        // and provided no meaningful benefit at sensor rates (~10 Hz).
        emit poseChanged(ahrsData.yaw, ahrsData.pitch, ahrsData.roll);
        emit positionChanged(ahrsData.lat, ahrsData.lon, ahrsData.altitude);
        emit altitudeChanged(ahrsData.altitude, ahrsData.altitudeMSL);
        emit statusChanged(ahrsData.siv, ahrsData.fixType, ahrsData.rtkType);
        emit headingChanged(ahrsData.heading);
        emit groundSpeedChanged(ahrsData.ground_speed);
        emit precisionChanged(ahrsData.pdop);
        emit positionECEFChanged(ahrsData.x, ahrsData.y, ahrsData.z);
        emit speedChanged(ahrsData.vx, ahrsData.vy, ahrsData.vz);
    }
    else if (lineTokens[0].startsWith("$GP")) {
        //@@TODO
    }
    current = ahrsData;
    emit redrawDisplay();
}

// Serial transport

void AHRSReceiver::run()
{
    if (mode == TCP) { runTCP(); return; }

    QSerialPort serialPort;
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
        serialPort.setDataTerminalReady(true);
        serialPort.waitForReadyRead(1000);

        QByteArray readData(1024, '0');
        done = false;

        while (!done) {
            std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
            readData = serialPort.readAll();
            while (serialPort.waitForReadyRead(10) && !readData.contains('$')) {
                serialPort.readAll();
            }

            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            while (serialPort.waitForReadyRead(100) && !readData.contains('\r')) {
                readData += serialPort.readAll();
            }

            if (readData.size() < 15)
                continue;

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            double syncTime = std::chrono::duration_cast<std::chrono::microseconds>(begin - t0).count();
            double readLineTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

            if (serialPort.error() == QSerialPort::ReadError) {
                qDebug() << QObject::tr("Failed to read from port %1, error: %2").arg(deviceName).arg(serialPort.errorString()) << endl;
                done = true;
                break;
            }
            else if (serialPort.error() == QSerialPort::TimeoutError && readData.isEmpty()) {
                std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
                double dataEmptyTime = std::chrono::duration_cast<std::chrono::microseconds>(end2 - end).count();
                qDebug() << QObject::tr("No data was currently available for reading from port %1").arg(deviceName) << "st:" << syncTime << ",et: " << dataEmptyTime << ",rt:" << readLineTime << ",bytes:" << readData.size() << endl;
                serialPort.waitForReadyRead(100);
            }

            parseLine(readData, current);
        }

    }
    catch (QException e) {
        qDebug() << e.what();
    }
    serialPort.close();
    qDebug() << QThread::currentThread() << " done";
    done = true;
}

// TCP transport

void AHRSReceiver::runTCP()
{
    QTcpSocket socket;
    AHRSData current;

    qDebug() << "AHRS TCP receiver connecting to" << tcpHost << ":" << tcpPort << "on" << QThread::currentThread();
    socket.connectToHost(tcpHost, tcpPort);
    if (!socket.waitForConnected(5000)) {
        qDebug() << "Failed to connect to TCP server:" << socket.errorString();
        done = true;
        return;
    }
    qDebug() << "TCP connected to" << tcpHost << ":" << tcpPort;
    done = false;

    QByteArray readData;
    while (!done) {
        // Sync to '$'
        readData = socket.readAll();
        while (socket.waitForReadyRead(10) && !readData.contains('$')) {
            readData = socket.readAll();
        }
        // Read full line ending with '\r'
        while (socket.waitForReadyRead(100) && !readData.contains('\r')) {
            readData += socket.readAll();
        }

        if (readData.size() < 15)
            continue;

        if (socket.state() != QAbstractSocket::ConnectedState) {
            qDebug() << "TCP connection lost";
            break;
        }

        parseLine(readData, current);
    }

    socket.disconnectFromHost();
    qDebug() << QThread::currentThread() << " TCP done";
    done = true;
}


// AHRSWidget

AHRSWidget::AHRSWidget(QWidget* parent)
    : QWidget(parent),
    deviceName("none")
{
    QVBoxLayout* vl = new QVBoxLayout(this);
    setLayout(vl);
    setFocusPolicy(Qt::NoFocus);

    masi        = new MASI(this);
    attitude    = new AHRSAttitudeIndicator(this);
    heading     = new AHRSCompass(this);
    info        = new AHRSInfo(this);
    deviceLabel = new QLabel(this);

    m_units         = Imperial;
    m_ahrsFont      = QFont("Arial", 8);
    m_ahrsColor     = QColor(Qt::white);
    m_statusFont    = QFont("Arial", 12);
    m_statusColor   = QColor(0, 255, 0);
    m_statusLargePx = 16;
    m_windowScale   = 1.0;
    m_lastAlt       = 0.0;
    m_lastAltValid  = false;
    m_altPanel      = nullptr;   // guard: rebuildAltPanelStyle() is called before the panel is created
    m_flightPanel   = nullptr;   // guard: rebuildFlightPanelStyle() likewise

    // Flight statistics state
    m_maxAltM      = 0.0;
    m_maxSpeedMs   = 0.0;
    m_maxGforce    = 0.0;
    m_tApogeeS     = 0.0;
    m_flightActive = false;
    m_prevVx = m_prevVy = m_prevVz = 0.0;

    // Altitude / climb panel
    m_altLabel   = new QLabel("-- ft", this);
    m_climbLabel = new QLabel("-- ft/s", this);
    m_altLabel->setAlignment(Qt::AlignCenter);
    m_climbLabel->setAlignment(Qt::AlignCenter);
    rebuildAltPanelStyle();

    auto makeAltHdr = [&](const QString &text) -> QLabel* {
        QLabel *l = new QLabel(text, this);
        l->setAlignment(Qt::AlignCenter);
        l->setStyleSheet("color: #666666; font-size: 11px; font-family: Arial; background: transparent;");
        return l;
    };

    m_altPanel = new QWidget(this);
    m_altPanel->setStyleSheet("background-color: black;");
    m_altPanel->setMinimumWidth(180);
    QVBoxLayout *altVBox = new QVBoxLayout(m_altPanel);
    altVBox->setContentsMargins(6, 0, 6, 0);
    altVBox->setSpacing(3);
    altVBox->addStretch(2);
    altVBox->addWidget(makeAltHdr("ALTITUDE"));
    altVBox->addWidget(m_altLabel);
    altVBox->addSpacing(14);
    altVBox->addWidget(makeAltHdr("CLIMB RATE"));
    altVBox->addWidget(m_climbLabel);
    altVBox->addStretch(2);

    // Flight statistics panel (right side of info bar, mirrors altPanel style)
    auto makeFlightHdr = [&](const QString &text) -> QLabel* {
        QLabel *l = new QLabel(text, this);
        l->setAlignment(Qt::AlignCenter);
        l->setStyleSheet("color: #666666; font-size: 11px; font-family: Arial; background: transparent;");
        return l;
    };

    m_maxAltValLabel   = new QLabel("--", this);
    m_maxSpeedValLabel = new QLabel("--", this);
    m_maxGValLabel     = new QLabel("--", this);
    m_tApogeeValLabel  = new QLabel("--", this);
    m_maxAltValLabel  ->setAlignment(Qt::AlignCenter);
    m_maxSpeedValLabel->setAlignment(Qt::AlignCenter);
    m_maxGValLabel    ->setAlignment(Qt::AlignCenter);
    m_tApogeeValLabel ->setAlignment(Qt::AlignCenter);

    m_flightPanel = new QWidget(this);
    m_flightPanel->setObjectName("flightPanel");
    m_flightPanel->setStyleSheet("QWidget#flightPanel { background-color: black; "
                                 "border-left:  1px solid #333333; "
                                 "border-right: 1px solid #333333; }");
    m_flightPanel->setMinimumWidth(150);
    QVBoxLayout *flightVBox = new QVBoxLayout(m_flightPanel);
    flightVBox->setContentsMargins(6, 0, 6, 0);
    flightVBox->setSpacing(2);
    flightVBox->addStretch(2);
    flightVBox->addWidget(makeFlightHdr("MAX ALTITUDE"));
    flightVBox->addWidget(m_maxAltValLabel);
    flightVBox->addSpacing(6);
    flightVBox->addWidget(makeFlightHdr("MAX SPEED"));
    flightVBox->addWidget(m_maxSpeedValLabel);
    flightVBox->addSpacing(6);
    flightVBox->addWidget(makeFlightHdr("MAX G"));
    flightVBox->addWidget(m_maxGValLabel);
    flightVBox->addSpacing(6);
    flightVBox->addWidget(makeFlightHdr("TIME TO APOGEE"));
    flightVBox->addWidget(m_tApogeeValLabel);
    flightVBox->addStretch(2);

    rebuildFlightPanelStyle();

    // Instrument panel: MASI left / attitude centre / compass right
    QWidget *instrPanel = new QWidget(this);
    instrPanel->setStyleSheet("background-color: #111111;");
    QHBoxLayout* instrRow = new QHBoxLayout(instrPanel);
    instrRow->addWidget(masi,     1);
    instrRow->addWidget(attitude, 1);
    instrRow->addWidget(heading,  1);
    instrRow->setSpacing(0);
    instrRow->setContentsMargins(2, 2, 2, 2);

    // Info panel: GPS info left | altPanel centred | right space
    QWidget *infoPanel = new QWidget(this);
    infoPanel->setStyleSheet("background-color: black;");
    // Both widgets share the single full-width cell.
    // info is left-aligned; m_altPanel is centered in the full bar width.
    QGridLayout* infoGrid = new QGridLayout(infoPanel);
    infoGrid->setContentsMargins(4, 2, 4, 2);
    infoGrid->addWidget(info,          0, 0);                                      // fills full cell
    infoGrid->addWidget(m_altPanel,    0, 0, Qt::AlignHCenter | Qt::AlignVCenter); // centered overlay
    infoGrid->addWidget(m_flightPanel, 0, 0, Qt::AlignRight   | Qt::AlignVCenter); // right overlay

    // Device label: compact, left-aligned
    deviceLabel->setText(deviceName);
    deviceLabel->setStyleSheet("color: #aaaaaa; font-size: 11px; background-color: black;");
    deviceLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);

    vl->addWidget(instrPanel, 1);                   // fills all available vertical space
    vl->addWidget(infoPanel,  0);                   // fixed preferred height
    vl->addWidget(deviceLabel, 0, Qt::AlignLeft);   // compact, left-aligned
    vl->setContentsMargins(0, 0, 0, 0);
    vl->setSpacing(0);

    // Load persisted appearance settings (no-op on first run -- defaults already set above)
    QString iniPath = QCoreApplication::applicationDirPath() + "/AHRS.ini";
    QSettings settings(iniPath, QSettings::IniFormat);

    settings.beginGroup("AHRS");
    if (settings.contains("font_family")) {
        QFont f;
        f.setFamily(settings.value("font_family").toString());
        f.setPointSize(settings.value("font_size", 8).toInt());
        f.setBold(settings.value("font_bold", false).toBool());
        f.setItalic(settings.value("font_italic", false).toBool());
        setAHRSFont(f);
        setAHRSColor(QColor(settings.value("color", "#ffffff").toString()));
    }
    settings.endGroup();

    settings.beginGroup("Status");
    if (settings.contains("font_family")) {
        QFont f;
        f.setFamily(settings.value("font_family").toString());
        f.setPointSize(settings.value("font_size", 12).toInt());
        f.setBold(settings.value("font_bold", false).toBool());
        f.setItalic(settings.value("font_italic", false).toBool());
        setStatusFont(f);
        setStatusColor(QColor(settings.value("color", "#00ff00").toString()));
    }
    settings.endGroup();

    settings.beginGroup("Display");
    {
        Units u = settings.value("units", "Imperial").toString() == "Metric" ? Metric : Imperial;
        m_units = u;
        masi->setUnits(u == Imperial ? MASI::Imperial : MASI::Metric);
        heading->setUnits(u == Imperial ? AHRSCompass::Imperial : AHRSCompass::Metric);
        info->setUnits(u == Imperial ? AHRSInfo::Imperial : AHRSInfo::Metric);
        m_altLabel->setText(u == Imperial ? "-- ft" : "-- m");
        m_climbLabel->setText(u == Imperial ? "-- ft/s" : "-- m/s");
    }
    settings.endGroup();
}

void AHRSWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    m_windowScale = qBound(0.4, height() / 600.0, 2.0);
    masi->setDisplayScale(m_windowScale);
    attitude->setDisplayScale(m_windowScale);
    heading->setDisplayScale(m_windowScale);
    info->setFontScale((m_statusLargePx / 16.0) * m_windowScale);
    rebuildAltPanelStyle();
}

void AHRSWidget::updatePose(double yaw, double pitch, double roll)
{
    heading->setYaw(yaw);
    attitude->setPitch(pitch);
    attitude->setRoll(roll);
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

    // Altitude display
    if (m_units == Imperial)
        m_altLabel->setText(QString("%1 ft").arg(qRound(altitude * 3.28084)));
    else
        m_altLabel->setText(QString("%1 m").arg(qRound(altitude)));

    // Climb rate from elapsed time and altitude delta
    if (m_lastAltValid) {
        double dt = m_climbTimer.elapsed() / 1000.0;
        if (dt >= 0.05) {
            double climbMs = (altitude - m_lastAlt) / dt;

            double climbDisplay, threshold;
            QString unit;
            if (m_units == Imperial) {
                climbDisplay = climbMs * 3.28084;
                threshold    = 0.5;
                unit         = "ft/s";
            } else {
                climbDisplay = climbMs;
                threshold    = 0.15;
                unit         = "m/s";
            }

            QString sign = (climbDisplay >= 0.0) ? "+" : "";
            m_climbLabel->setText(QString("%1%2 %3").arg(sign).arg(climbDisplay, 0, 'f', 1).arg(unit));

            QString color = (climbDisplay > threshold)  ? m_statusColor.name() :
                            (climbDisplay < -threshold) ? "#ff4444" : "#ffff00";
            int clPx = qMax(6, qRound((m_statusLargePx + 4) * m_windowScale));
            m_climbLabel->setStyleSheet(
                QString("color: %1; font-size: %2px; font-family: %3; background: transparent;")
                .arg(color).arg(clPx).arg(m_statusFont.family()));

            m_lastAlt = altitude;
            m_climbTimer.restart();
        }
    } else {
        m_lastAlt      = altitude;
        m_lastAltValid = true;
        m_climbTimer.start();
    }

    // Flight stats: track max altitude and time-to-apogee
    if (m_flightActive && altitude > m_maxAltM) {
        m_maxAltM  = altitude;
        m_tApogeeS = m_flightTimer.elapsed() / 1000.0;
        if (m_units == Imperial)
            m_maxAltValLabel->setText(QString("%1 ft").arg(qRound(m_maxAltM * 3.28084)));
        else
            m_maxAltValLabel->setText(QString("%1 m").arg(qRound(m_maxAltM)));
        m_tApogeeValLabel->setText(QString("%1 s").arg(m_tApogeeS, 0, 'f', 1));
    }
}

void AHRSWidget::setUnits(Units u)
{
    m_units = u;
    masi->setUnits(u == Imperial ? MASI::Imperial : MASI::Metric);
    heading->setUnits(u == Imperial ? AHRSCompass::Imperial : AHRSCompass::Metric);
    info->setUnits(u == Imperial ? AHRSInfo::Imperial : AHRSInfo::Metric);

    QString iniPath = QCoreApplication::applicationDirPath() + "/AHRS.ini";
    QSettings settings(iniPath, QSettings::IniFormat);
    settings.beginGroup("Display");
    settings.setValue("units", u == Imperial ? "Imperial" : "Metric");
    settings.endGroup();
    // Placeholder text while waiting for next packet
    if (!m_lastAltValid) {
        m_altLabel->setText(u == Imperial ? "-- ft" : "-- m");
        m_climbLabel->setText(u == Imperial ? "-- ft/s" : "-- m/s");
    }
    // Refresh stored max values in new units
    if (m_maxAltM > 0.0) {
        if (u == Imperial)
            m_maxAltValLabel->setText(QString("%1 ft").arg(qRound(m_maxAltM * 3.28084)));
        else
            m_maxAltValLabel->setText(QString("%1 m").arg(qRound(m_maxAltM)));
    }
    if (m_maxSpeedMs > 0.0) {
        double displaySpeed = (u == Imperial) ? m_maxSpeedMs * 3.28084 : m_maxSpeedMs;
        QString unit        = (u == Imperial) ? "Ft/Sec" : "m/s";
        m_maxSpeedValLabel->setText(QString("%1 %2").arg(displaySpeed, 0, 'f', 1).arg(unit));
    }
}

void AHRSWidget::updateGroundSpeed(double ground_speed)
{
    // No ground-speed display widget yet -- data received but not shown.
    Q_UNUSED(ground_speed);
}

void AHRSWidget::updateSpeed(double vx, double vy, double vz)
{
    info->ecefVX->setText("Vx:" + QString::number(vx, 'f', 2));
    info->ecefVY->setText("Vy:" + QString::number(vy, 'f', 2));
    info->ecefVZ->setText("Vz:" + QString::number(vz, 'f', 2));

    double speedMs = sqrt(vx*vx + vy*vy + vz*vz);

    // Speed of sound: rough linear with altitude (valid below ~11 km)
    // -6.5 K/km lapse rate -> approx -4 m/s per 1000 m altitude
    double altM = m_lastAltValid ? m_lastAlt : 0.0;
    double sos  = qMax(295.0, 340.29 - 0.004 * altM);

    double masiSpeed = (m_units == Imperial) ? speedMs * 3.28084 : speedMs;
    masi->setSpeed(masiSpeed);
    masi->setMach(speedMs / sos);

    // G-force from 3-axis velocity delta — computed every packet, pre- and post-launch
    double gforce = 0.0;
    if (m_gTimer.isValid()) {
        double dt_g = m_gTimer.elapsed() / 1000.0;
        if (dt_g > 0.01) {
            double ax = (vx - m_prevVx) / dt_g;
            double ay = (vy - m_prevVy) / dt_g;
            double az = (vz - m_prevVz) / dt_g;
            gforce = sqrt(ax*ax + ay*ay + az*az) / 9.80665;
        }
    }
    m_gTimer.restart();
    m_prevVx = vx;  m_prevVy = vy;  m_prevVz = vz;

    // Launch detection: sustained >= 2 G for >= 500 ms
    if (!m_flightActive) {
        if (gforce >= 2.0) {
            if (!m_launchGTimer.isValid())
                m_launchGTimer.start();
            else if (m_launchGTimer.elapsed() >= 500) {
                m_flightActive = true;
                m_flightTimer.start();
            }
        } else {
            m_launchGTimer.invalidate();   // G dropped — reset the window
        }
    }

    // Track max speed and max G after launch
    if (m_flightActive) {
        if (speedMs > m_maxSpeedMs) {
            m_maxSpeedMs = speedMs;
            double displaySpeed = (m_units == Imperial) ? m_maxSpeedMs * 3.28084 : m_maxSpeedMs;
            QString unit        = (m_units == Imperial) ? "Ft/Sec" : "m/s";
            m_maxSpeedValLabel->setText(QString("%1 %2").arg(displaySpeed, 0, 'f', 1).arg(unit));
        }
        if (gforce > m_maxGforce) {
            m_maxGforce = gforce;
            m_maxGValLabel->setText(QString("%1 G").arg(m_maxGforce, 0, 'f', 1));
        }
    }
}

void AHRSWidget::updateHeading(double bearing)
{
    // GPS compass bearing is more reliable than INS yaw for the compass display.
    // headingChanged fires after poseChanged in parseLine(), so this wins.
    heading->setYaw(bearing);
}

void AHRSWidget::updateEstimatedPosition(double lat, double lon, double altMslM)
{
    info->updateDRPosition(lat, lon, altMslM);
}

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
    masi->update();
    attitude->update();
    heading->update();
}

void AHRSWidget::rebuildAltPanelStyle()
{
    QString fam   = m_statusFont.family();
    int     altPx = qMax(6, qRound((m_statusLargePx + 10) * m_windowScale));
    int     clPx  = qMax(6, qRound((m_statusLargePx + 4)  * m_windowScale));
    m_altLabel->setStyleSheet(
        QString("color: %1; font-size: %2px; font-weight: bold; font-family: %3; background: transparent;")
        .arg(m_statusColor.name()).arg(altPx).arg(fam));
    m_climbLabel->setStyleSheet(
        QString("color: %1; font-size: %2px; font-family: %3; background: transparent;")
        .arg(m_statusColor.name()).arg(clPx).arg(fam));

    // Size the panel to fit the widest expected text at the current font scale.
    // "99999 ft/s" covers both altitude ("99999 ft") and climb ("-999.9 ft/s").
    if (m_altPanel) {
        QFont altFont(fam);  altFont.setPixelSize(altPx);  altFont.setBold(true);
        QFont clFont(fam);   clFont.setPixelSize(clPx);
        int needed = qMax(QFontMetrics(altFont).horizontalAdvance("99999 ft"),
                          QFontMetrics(clFont) .horizontalAdvance("-9999.9 ft/s"));
        int panelW = qMax(180, needed + 40);   // 20 px margin on each side
        m_altPanel->setFixedWidth(panelW);
    }
    rebuildFlightPanelStyle();
}

void AHRSWidget::rebuildFlightPanelStyle()
{
    if (!m_flightPanel) return;
    QString fam   = m_statusFont.family();
    int     valPx = qMax(6, qRound((m_statusLargePx + 2) * m_windowScale));
    QString style = QString("color: %1; font-size: %2px; font-family: %3; background: transparent;")
        .arg(m_statusColor.name()).arg(valPx).arg(fam);
    m_maxAltValLabel  ->setStyleSheet(style);
    m_maxSpeedValLabel->setStyleSheet(style);
    m_maxGValLabel    ->setStyleSheet(style);
    m_tApogeeValLabel ->setStyleSheet(style);

    QFont valFont(fam);  valFont.setPixelSize(valPx);
    int needed = QFontMetrics(valFont).horizontalAdvance("999999.9 Ft/Sec");
    m_flightPanel->setFixedWidth(qMax(200, needed + 40));
}

void AHRSWidget::resetFlightData()
{
    m_maxAltM      = 0.0;
    m_maxSpeedMs   = 0.0;
    m_maxGforce    = 0.0;
    m_tApogeeS     = 0.0;
    m_flightActive = false;
    m_prevVx = m_prevVy = m_prevVz = 0.0;
    m_gTimer.invalidate();
    m_launchGTimer.invalidate();
    if (m_maxAltValLabel)   m_maxAltValLabel  ->setText("--");
    if (m_maxSpeedValLabel) m_maxSpeedValLabel->setText("--");
    if (m_maxGValLabel)     m_maxGValLabel    ->setText("--");
    if (m_tApogeeValLabel)  m_tApogeeValLabel ->setText("--");
}

void AHRSWidget::setAHRSFont(const QFont &font)
{
    m_ahrsFont = font;
    attitude->setInstrFont(font);
    heading->setInstrFont(font);
}

void AHRSWidget::setAHRSColor(const QColor &color)
{
    m_ahrsColor = color;
    attitude->setLabelColor(color);
    heading->setLabelColor(color);
}

void AHRSWidget::setStatusFont(const QFont &font)
{
    m_statusFont    = font;
    m_statusLargePx = qRound(font.pointSize() * 4.0 / 3.0);
    info->setFontFamily(font.family());
    info->setFontScale((m_statusLargePx / 16.0) * m_windowScale);
    rebuildAltPanelStyle();
}

void AHRSWidget::setStatusColor(const QColor &color)
{
    m_statusColor = color;
    info->setTextColor(color);
    rebuildAltPanelStyle();
}

void AHRSWidget::setReceiver(AHRSReceiver* receiver)
{
    resetFlightData();
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
    connect(receiver, SIGNAL(estimatedPositionChanged(double, double, double)), this, SLOT(updateEstimatedPosition(double, double, double)));
    connect(receiver, SIGNAL(redrawDisplay()), this, SLOT(updateHUD()));
}
