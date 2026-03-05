#include "emulator_window.h"
#include "emulator_defaults.h"

#include <cmath>
#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QWidget>
#include <QMessageBox>
#include <QHostAddress>

// ─── Constructor / Destructor ────────────────────────────────────────────────

EmulatorWindow::EmulatorWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_server(nullptr)
    , m_running(false)
    , m_packetCount(0)
{
    setWindowTitle(EMUL_WINDOW_TITLE);
    setMinimumWidth(EMUL_MIN_WINDOW_WIDTH);

    m_timer   = new QTimer(this);
    m_vehicle = new VehicleRocket(this);

    connect(m_timer, &QTimer::timeout, this, &EmulatorWindow::onTick);

    buildUi();
    updateMotorDerived();   // sets params on vehicle, calls setup() + init()
}

EmulatorWindow::~EmulatorWindow()
{
    if (m_server && m_server->isListening())
        m_server->close();
}

// ─── UI construction ──────────────────────────────────────────────────────────

void EmulatorWindow::buildUi()
{
    QWidget *central = new QWidget(this);
    setCentralWidget(central);
    QVBoxLayout *mainLayout = new QVBoxLayout(central);

    // ── Network controls ──────────────────────────────────────────────────────
    QHBoxLayout *netRow = new QHBoxLayout;
    netRow->addWidget(new QLabel("TCP Port:"));
    m_tcpPortSpinBox = new QSpinBox;
    m_tcpPortSpinBox->setRange(EMUL_TCP_PORT_MIN, EMUL_TCP_PORT_MAX);
    m_tcpPortSpinBox->setValue(EMUL_DEFAULT_TCP_PORT);
    netRow->addWidget(m_tcpPortSpinBox);
    netRow->addSpacing(16);
    netRow->addWidget(new QLabel("Rate (ms):"));
    m_rateSpinBox = new QSpinBox;
    m_rateSpinBox->setRange(EMUL_RATE_MIN_MS, EMUL_RATE_MAX_MS);
    m_rateSpinBox->setValue(EMUL_DEFAULT_RATE_MS);
    m_rateSpinBox->setSingleStep(10);
    connect(m_rateSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            m_timer, QOverload<int>::of(&QTimer::setInterval));
    netRow->addWidget(m_rateSpinBox);
    netRow->addStretch();
    mainLayout->addLayout(netRow);

    // ── Rocket parameters group ───────────────────────────────────────────────
    QGroupBox   *rocketGroup = new QGroupBox("Rocket Parameters");
    QGridLayout *rg          = new QGridLayout(rocketGroup);

    // Row 0: Motor | Loaded Weight
    rg->addWidget(new QLabel("Motor:"), 0, 0);
    m_motorEdit = new QLineEdit(EMUL_DEFAULT_MOTOR);
    m_motorEdit->setMaximumWidth(80);
    m_motorEdit->setPlaceholderText("e.g. N1000");
    connect(m_motorEdit, &QLineEdit::textChanged, this, &EmulatorWindow::onMotorChanged);
    rg->addWidget(m_motorEdit, 0, 1);

    rg->addWidget(new QLabel("Loaded Wt (lbs):"), 0, 2);
    m_loadedWtSpinBox = new QDoubleSpinBox;
    m_loadedWtSpinBox->setRange(EMUL_LOADED_WT_MIN_LBS, EMUL_LOADED_WT_MAX_LBS);
    m_loadedWtSpinBox->setValue(EMUL_DEFAULT_LOADED_WT_LBS);
    m_loadedWtSpinBox->setDecimals(1);
    m_loadedWtSpinBox->setSingleStep(0.5);
    connect(m_loadedWtSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &EmulatorWindow::updateMotorDerived);
    rg->addWidget(m_loadedWtSpinBox, 0, 3);

    // Row 1: Prop Weight | Launch Angle
    rg->addWidget(new QLabel("Prop Wt (lbs):"), 1, 0);
    m_propWtSpinBox = new QDoubleSpinBox;
    m_propWtSpinBox->setRange(EMUL_PROP_WT_MIN_LBS, EMUL_PROP_WT_MAX_LBS);
    m_propWtSpinBox->setValue(EMUL_DEFAULT_PROP_WT_LBS);
    m_propWtSpinBox->setDecimals(1);
    m_propWtSpinBox->setSingleStep(0.5);
    connect(m_propWtSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &EmulatorWindow::updateMotorDerived);
    rg->addWidget(m_propWtSpinBox, 1, 1);

    rg->addWidget(new QLabel("Launch Angle (° from vertical):"), 1, 2);
    m_launchAngleSpinBox = new QDoubleSpinBox;
    m_launchAngleSpinBox->setRange(EMUL_LAUNCH_ANGLE_MIN_DEG, EMUL_LAUNCH_ANGLE_MAX_DEG);
    m_launchAngleSpinBox->setValue(EMUL_DEFAULT_LAUNCH_ANGLE_DEG);
    m_launchAngleSpinBox->setDecimals(1);
    m_launchAngleSpinBox->setSingleStep(1.0);
    rg->addWidget(m_launchAngleSpinBox, 1, 3);

    // Row 2: Launch altitude | Cd
    rg->addWidget(new QLabel("Launch Alt (ft):"), 2, 0);
    m_launchAltSpinBox = new QSpinBox;
    m_launchAltSpinBox->setRange(EMUL_LAUNCH_ALT_MIN_FT, EMUL_LAUNCH_ALT_MAX_FT);
    m_launchAltSpinBox->setValue(EMUL_DEFAULT_LAUNCH_ALT_FT);
    m_launchAltSpinBox->setSingleStep(100);
    rg->addWidget(m_launchAltSpinBox, 2, 1);

    rg->addWidget(new QLabel("Drag Coeff (Cd):"), 2, 2);
    m_cdSpinBox = new QDoubleSpinBox;
    m_cdSpinBox->setRange(EMUL_CD_MIN, EMUL_CD_MAX);
    m_cdSpinBox->setValue(EMUL_DEFAULT_CD);
    m_cdSpinBox->setDecimals(2);
    m_cdSpinBox->setSingleStep(0.05);
    rg->addWidget(m_cdSpinBox, 2, 3);

    // Row 3: Skin friction coefficient
    rg->addWidget(new QLabel("Skin Friction (Cf):"), 3, 0);
    m_cfSpinBox = new QDoubleSpinBox;
    m_cfSpinBox->setRange(EMUL_CF_MIN, EMUL_CF_MAX);
    m_cfSpinBox->setValue(EMUL_DEFAULT_CF);
    m_cfSpinBox->setDecimals(4);
    m_cfSpinBox->setSingleStep(0.001);
    m_cfSpinBox->setToolTip(EMUL_CF_TOOLTIP);
    rg->addWidget(m_cfSpinBox, 3, 1);

    // Row 4: Diameter | Length
    rg->addWidget(new QLabel("Body Diameter (in):"), 4, 0);
    m_diameterSpinBox = new QDoubleSpinBox;
    m_diameterSpinBox->setRange(EMUL_DIAMETER_MIN_IN, EMUL_DIAMETER_MAX_IN);
    m_diameterSpinBox->setValue(EMUL_DEFAULT_DIAMETER_IN);
    m_diameterSpinBox->setDecimals(2);
    m_diameterSpinBox->setSingleStep(0.25);
    rg->addWidget(m_diameterSpinBox, 4, 1);

    rg->addWidget(new QLabel("Body Length (ft):"), 4, 2);
    m_lengthSpinBox = new QDoubleSpinBox;
    m_lengthSpinBox->setRange(EMUL_LENGTH_MIN_FT, EMUL_LENGTH_MAX_FT);
    m_lengthSpinBox->setValue(EMUL_DEFAULT_LENGTH_FT);
    m_lengthSpinBox->setDecimals(1);
    m_lengthSpinBox->setSingleStep(0.5);
    rg->addWidget(m_lengthSpinBox, 4, 3);

    // Row 5: Derived info (full width)
    m_derivedLabel = new QLabel("—");
    m_derivedLabel->setStyleSheet(QString("color: %1;").arg(EMUL_DERIVED_LABEL_COLOR));
    rg->addWidget(m_derivedLabel, 5, 0, 1, 4);

    rg->setColumnStretch(1, 1);
    rg->setColumnStretch(3, 1);
    mainLayout->addWidget(rocketGroup);

    // ── Parachute options group ───────────────────────────────────────────────
    QGroupBox   *chuteGroup  = new QGroupBox("Parachute");
    QHBoxLayout *chuteLayout = new QHBoxLayout(chuteGroup);

    m_drogueCheckBox = new QCheckBox(EMUL_DROGUE_LABEL);
    m_drogueCheckBox->setChecked(true);
    chuteLayout->addWidget(m_drogueCheckBox);

    chuteLayout->addSpacing(24);

    m_mainChuteCheckBox = new QCheckBox(EMUL_MAIN_CHUTE_LABEL);
    m_mainChuteCheckBox->setChecked(true);
    chuteLayout->addWidget(m_mainChuteCheckBox);

    m_mainChuteAltLabel = new QLabel("Deploy Alt (ft):");
    chuteLayout->addWidget(m_mainChuteAltLabel);

    m_mainChuteAltSpinBox = new QSpinBox;
    m_mainChuteAltSpinBox->setRange(EMUL_MAIN_CHUTE_ALT_MIN_FT, EMUL_MAIN_CHUTE_ALT_MAX_FT);
    m_mainChuteAltSpinBox->setValue(EMUL_DEFAULT_MAIN_CHUTE_ALT_FT);
    m_mainChuteAltSpinBox->setSingleStep(100);
    m_mainChuteAltSpinBox->setSuffix(" ft AGL");
    chuteLayout->addWidget(m_mainChuteAltSpinBox);

    chuteLayout->addStretch();

    auto updateMainChuteAltState = [this]() {
        bool en = m_mainChuteCheckBox->isChecked();
        m_mainChuteAltLabel->setEnabled(en);
        m_mainChuteAltSpinBox->setEnabled(en);
    };
    connect(m_mainChuteCheckBox, &QCheckBox::stateChanged,
            [updateMainChuteAltState](int){ updateMainChuteAltState(); });
    updateMainChuteAltState();

    mainLayout->addWidget(chuteGroup);

    // ── Start/Stop button ─────────────────────────────────────────────────────
    m_startStopBtn = new QPushButton("Start Server");
    m_startStopBtn->setMinimumHeight(EMUL_START_BTN_MIN_HEIGHT);
    QFont btnFont = m_startStopBtn->font();
    btnFont.setPointSize(EMUL_START_BTN_FONT_PT);
    btnFont.setBold(true);
    m_startStopBtn->setFont(btnFont);
    m_startStopBtn->setStyleSheet(EMUL_BTN_STYLE_START);
    connect(m_startStopBtn, &QPushButton::clicked, this, &EmulatorWindow::onStartStop);
    mainLayout->addWidget(m_startStopBtn);

    // ── Status label ──────────────────────────────────────────────────────────
    m_statusLabel = new QLabel("Status: Stopped");
    mainLayout->addWidget(m_statusLabel);

    // ── Live values group ─────────────────────────────────────────────────────
    QGroupBox   *liveGroup = new QGroupBox("Live Transmitted Values");
    QGridLayout *grid      = new QGridLayout(liveGroup);

    auto makeValueLabel = [&]() {
        QLabel *l = new QLabel("—");
        l->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        return l;
    };

    m_phaseLabel   = makeValueLabel();
    m_yawLabel     = makeValueLabel();
    m_pitchLabel   = makeValueLabel();
    m_rollLabel    = makeValueLabel();
    m_headingLabel = makeValueLabel();
    m_latLabel     = makeValueLabel();
    m_lonLabel     = makeValueLabel();
    m_altLabel     = makeValueLabel();
    m_mslLabel     = makeValueLabel();
    m_sivLabel     = makeValueLabel();
    m_fixLabel     = makeValueLabel();
    m_rtkLabel     = makeValueLabel();
    m_pdopLabel    = makeValueLabel();
    m_speedLabel   = makeValueLabel();
    m_ecefLabel    = makeValueLabel();
    m_velLabel     = makeValueLabel();
    m_packetLabel  = makeValueLabel();

    int row = 0;
    grid->addWidget(new QLabel("Phase:"),    row, 0); grid->addWidget(m_phaseLabel,   row, 1);
    grid->addWidget(new QLabel("Yaw:"),      row, 2); grid->addWidget(m_yawLabel,     row, 3);
    ++row;
    grid->addWidget(new QLabel("Pitch:"),    row, 0); grid->addWidget(m_pitchLabel,   row, 1);
    grid->addWidget(new QLabel("Roll:"),     row, 2); grid->addWidget(m_rollLabel,    row, 3);
    ++row;
    grid->addWidget(new QLabel("Heading:"),  row, 0); grid->addWidget(m_headingLabel, row, 1);
    grid->addWidget(new QLabel("Lat:"),      row, 2); grid->addWidget(m_latLabel,     row, 3);
    ++row;
    grid->addWidget(new QLabel("Lon:"),      row, 0); grid->addWidget(m_lonLabel,     row, 1);
    grid->addWidget(new QLabel("Alt:"),      row, 2); grid->addWidget(m_altLabel,     row, 3);
    ++row;
    grid->addWidget(new QLabel("MSL:"),      row, 0); grid->addWidget(m_mslLabel,     row, 1);
    grid->addWidget(new QLabel("SIV:"),      row, 2); grid->addWidget(m_sivLabel,     row, 3);
    ++row;
    grid->addWidget(new QLabel("Fix:"),      row, 0); grid->addWidget(m_fixLabel,     row, 1);
    grid->addWidget(new QLabel("RTK:"),      row, 2); grid->addWidget(m_rtkLabel,     row, 3);
    ++row;
    grid->addWidget(new QLabel("PDOP:"),     row, 0); grid->addWidget(m_pdopLabel,    row, 1);
    grid->addWidget(new QLabel("Speed:"),    row, 2); grid->addWidget(m_speedLabel,   row, 3);
    ++row;
    grid->addWidget(new QLabel("ECEF:"),     row, 0); grid->addWidget(m_ecefLabel,    row, 1, 1, 3);
    ++row;
    grid->addWidget(new QLabel("Velocity:"), row, 0); grid->addWidget(m_velLabel,     row, 1, 1, 3);
    ++row;
    grid->addWidget(new QLabel("Packets:"),  row, 0); grid->addWidget(m_packetLabel,  row, 1);

    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(3, 1);
    mainLayout->addWidget(liveGroup);

    // ── Last line (monospace, full width) ─────────────────────────────────────
    QGroupBox   *lastGroup  = new QGroupBox("Last Transmitted Line");
    QVBoxLayout *lastLayout = new QVBoxLayout(lastGroup);
    m_lastLineLabel = new QLabel("(none)");
    QFont mono(EMUL_MONO_FONT_FAMILY, EMUL_MONO_FONT_SIZE_PT);
    m_lastLineLabel->setFont(mono);
    m_lastLineLabel->setWordWrap(true);
    lastLayout->addWidget(m_lastLineLabel);
    mainLayout->addWidget(lastGroup);
}

// ─── Motor designation parser / derived label ────────────────────────────────

void EmulatorWindow::onMotorChanged(const QString &)
{
    updateMotorDerived();
}

void EmulatorWindow::updateMotorDerived()
{
    QString motor = m_motorEdit->text().trimmed();
    if (motor.isEmpty()) {
        m_derivedLabel->setText("—");
        return;
    }

    double impulse = VehicleRocket::traTotalImpulse(motor[0]);
    if (impulse <= 0.0) {
        m_derivedLabel->setText("Unknown motor class (use A–O)");
        return;
    }

    bool   ok;
    double thrust = motor.mid(1).toDouble(&ok);
    if (!ok || thrust <= 0.0) {
        m_derivedLabel->setText("Invalid average thrust after letter");
        return;
    }

    // Build full parameter set from current UI and push to vehicle.
    // UI weight inputs are in lbs; physics model requires kg — multiply by LBS_TO_KG.
    RocketParams p;
    p.totalImpulseNs = impulse;
    p.avgThrustN     = thrust;
    p.loadedMassKg   = m_loadedWtSpinBox->value()   * LBS_TO_KG;
    p.propMassKg     = m_propWtSpinBox->value()      * LBS_TO_KG;
    p.launchAngleDeg = m_launchAngleSpinBox->value();
    p.launchAltFt    = m_launchAltSpinBox->value();
    p.cd             = m_cdSpinBox->value();
    p.cf             = m_cfSpinBox->value();
    p.diameterIn     = m_diameterSpinBox->value();
    p.lengthFt       = m_lengthSpinBox->value();
    p.launchLatDeg   = EMUL_LAUNCH_LAT_DEG;
    p.launchLonDeg   = EMUL_LAUNCH_LON_DEG;
    p.drogueChute    = m_drogueCheckBox->isChecked();
    p.mainChute      = m_mainChuteCheckBox->isChecked();
    p.mainChuteAglFt = m_mainChuteAltSpinBox->value();

    m_vehicle->setParams(p);
    m_vehicle->setup();

    m_derivedLabel->setText(
        QString("Impulse: %1 Ns  |  Burn: %2 s  |  Ve: %3 m/s  |  Isp: %4 s  |  Dry: %5 kg")
            .arg(impulse,                 0, 'f', 0)
            .arg(m_vehicle->burnTime(),   0, 'f', 2)
            .arg(m_vehicle->ve(),         0, 'f', 0)
            .arg(m_vehicle->isp(),        0, 'f', 0)
            .arg(m_vehicle->dryMassKg(),  0, 'f', 2));
}

// ─── Start / Stop ─────────────────────────────────────────────────────────────

void EmulatorWindow::onStartStop()
{
    if (m_running) {
        setRunning(false);
        return;
    }

    // Apply current UI parameters, then reset flight state
    updateMotorDerived();   // → vehicle->setParams() + setup()
    m_vehicle->init();      // reset to PRELAUNCH at launch site altitude

    quint16 port = static_cast<quint16>(m_tcpPortSpinBox->value());
    m_server = new QTcpServer(this);
    connect(m_server, &QTcpServer::newConnection, this, &EmulatorWindow::onNewConnection);

    if (!m_server->listen(QHostAddress::LocalHost, port)) {
        QMessageBox::critical(this, "Server Error",
            QString("Cannot listen on port %1:\n%2").arg(port).arg(m_server->errorString()));
        delete m_server;
        m_server = nullptr;
        return;
    }

    m_packetCount = 0;
    setRunning(true);
}

void EmulatorWindow::setRunning(bool running)
{
    m_running = running;
    if (running) {
        m_timer->start(m_rateSpinBox->value());
        m_startStopBtn->setText("Stop Server");
        m_startStopBtn->setStyleSheet(EMUL_BTN_STYLE_STOP);
        m_statusLabel->setText(QString("Status: Listening on port %1 — connect client to LAUNCH")
            .arg(m_tcpPortSpinBox->value()));
        // Lock all inputs during run
        m_tcpPortSpinBox->setEnabled(false);
        m_rateSpinBox->setEnabled(false);
        m_motorEdit->setEnabled(false);
        m_loadedWtSpinBox->setEnabled(false);
        m_propWtSpinBox->setEnabled(false);
        m_launchAngleSpinBox->setEnabled(false);
        m_launchAltSpinBox->setEnabled(false);
        m_cdSpinBox->setEnabled(false);
        m_cfSpinBox->setEnabled(false);
        m_diameterSpinBox->setEnabled(false);
        m_lengthSpinBox->setEnabled(false);
        m_drogueCheckBox->setEnabled(false);
        m_mainChuteCheckBox->setEnabled(false);
        m_mainChuteAltSpinBox->setEnabled(false);
        m_mainChuteAltLabel->setEnabled(false);
    } else {
        m_timer->stop();
        for (QTcpSocket *s : m_clients)
            s->disconnectFromHost();
        m_clients.clear();
        if (m_server) {
            m_server->close();
            delete m_server;
            m_server = nullptr;
        }
        m_startStopBtn->setText("Start Server");
        m_startStopBtn->setStyleSheet(EMUL_BTN_STYLE_START);
        m_statusLabel->setText("Status: Stopped");
        // Unlock inputs
        m_tcpPortSpinBox->setEnabled(true);
        m_rateSpinBox->setEnabled(true);
        m_motorEdit->setEnabled(true);
        m_loadedWtSpinBox->setEnabled(true);
        m_propWtSpinBox->setEnabled(true);
        m_launchAngleSpinBox->setEnabled(true);
        m_launchAltSpinBox->setEnabled(true);
        m_cdSpinBox->setEnabled(true);
        m_cfSpinBox->setEnabled(true);
        m_diameterSpinBox->setEnabled(true);
        m_lengthSpinBox->setEnabled(true);
        m_drogueCheckBox->setEnabled(true);
        m_mainChuteCheckBox->setEnabled(true);
        bool mainChecked = m_mainChuteCheckBox->isChecked();
        m_mainChuteAltSpinBox->setEnabled(mainChecked);
        m_mainChuteAltLabel->setEnabled(mainChecked);
    }
}

// ─── TCP connection management ────────────────────────────────────────────────

void EmulatorWindow::onNewConnection()
{
    while (m_server->hasPendingConnections()) {
        QTcpSocket *socket = m_server->nextPendingConnection();
        connect(socket, &QTcpSocket::disconnected, this, &EmulatorWindow::onClientDisconnected);
        m_clients.append(socket);

        // First client triggers launch
        m_vehicle->launch();
    }

    m_statusLabel->setText(QString("Status: %1 | port %2 | %3 client(s) @ %4 ms/packet")
        .arg(m_vehicle->phaseName())
        .arg(m_tcpPortSpinBox->value())
        .arg(m_clients.size())
        .arg(m_rateSpinBox->value()));
}

void EmulatorWindow::onClientDisconnected()
{
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    if (socket) {
        m_clients.removeAll(socket);
        socket->deleteLater();
    }
    if (m_running) {
        m_statusLabel->setText(QString("Status: %1 | port %2 | %3 client(s)")
            .arg(m_vehicle->phaseName())
            .arg(m_tcpPortSpinBox->value())
            .arg(m_clients.size()));
    }
}

// ─── Simulation tick ──────────────────────────────────────────────────────────

void EmulatorWindow::onTick()
{
    const double dt = m_rateSpinBox->value() / 1000.0;

    // ── 1. Advance vehicle simulation ────────────────────────────────────────
    m_vehicle->run(dt);

    // ── 2. Read telemetry state from vehicle ─────────────────────────────────
    double yaw        = m_vehicle->yaw();
    double pitch      = m_vehicle->pitch();
    double roll       = m_vehicle->roll();
    double heading    = yaw;
    double vx_ms      = m_vehicle->vxMs();
    double vy_ms      = m_vehicle->vyMs();
    double vz_ms      = m_vehicle->vz();
    double totalSpeed = sqrt(vx_ms*vx_ms + vy_ms*vy_ms + vz_ms*vz_ms);

    // Altitude in metres — physics loop tracks feet; convert for the wire packet
    double altM = m_vehicle->altFt() * EMUL_FT_TO_M;

    // ── 3. Build $INS wire packet ─────────────────────────────────────────────
    double dynLat    = m_vehicle->latDeg();
    double dynLon    = m_vehicle->lonDeg();
    qint64 lat_e7    = static_cast<qint64>(dynLat * 1e7);
    qint64 lon_e7    = static_cast<qint64>(dynLon * 1e7);
    qint64 alt_mm    = static_cast<qint64>(altM * 1000.0);
    qint64 msl_mm    = static_cast<qint64>(m_vehicle->altMslM() * 1000.0);
    qint64 gspeed_mm = static_cast<qint64>(m_vehicle->vh() * 1000.0);
    qint64 ecefX     = static_cast<qint64>(m_vehicle->ecefX());
    qint64 ecefY     = static_cast<qint64>(m_vehicle->ecefY());
    qint64 ecefZ     = static_cast<qint64>(m_vehicle->ecefZ());

    QString line = QString(
        "$INS,%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15,%16,%17,%18,%19\r")
        .arg(yaw,     0, 'f', 3)
        .arg(pitch,   0, 'f', 3)
        .arg(roll,    0, 'f', 3)
        .arg(lat_e7)
        .arg(lon_e7)
        .arg(alt_mm)
        .arg(msl_mm)
        .arg(EMUL_GPS_SIV)
        .arg(EMUL_GPS_FIX)
        .arg(EMUL_GPS_RTK)
        .arg(heading, 0, 'f', 3)
        .arg(gspeed_mm)
        .arg(EMUL_GPS_PDOP)
        .arg(ecefX)
        .arg(ecefY)
        .arg(ecefZ)
        .arg(vx_ms,   0, 'f', 4)
        .arg(vy_ms,   0, 'f', 4)
        .arg(vz_ms,   0, 'f', 4);

    // ── 4. Send to all connected clients ─────────────────────────────────────
    if (!m_clients.isEmpty()) {
        QByteArray data = line.toLocal8Bit();
        for (QTcpSocket *socket : m_clients)
            socket->write(data);
        ++m_packetCount;
    }

    // ── 5. Update live UI labels ──────────────────────────────────────────────
    static const char* fixNames[] = { "No Fix", "Dead Reck.", "2D Fix", "3D Fix", "GNSS+DR" };
    static const char* rtkNames[] = { "N/A", "RTK Float", "RTK Fix" };

    m_phaseLabel  ->setText(m_vehicle->phaseName());
    m_yawLabel    ->setText(QString("%1 deg").arg(yaw,       7, 'f', 2));
    m_pitchLabel  ->setText(QString("%1 deg").arg(pitch,     7, 'f', 2));
    m_rollLabel   ->setText(QString("%1 deg").arg(roll,      7, 'f', 2));
    m_headingLabel->setText(QString("%1 deg").arg(heading,   7, 'f', 2));
    m_latLabel    ->setText(QString("%1 deg").arg(dynLat,    0, 'f', 6));
    m_lonLabel    ->setText(QString("%1 deg").arg(dynLon,    0, 'f', 6));
    double mslM = m_vehicle->altMslM();
    m_altLabel    ->setText(QString("%1 ft  (%2 m)").arg(qRound(m_vehicle->altFt())).arg(altM, 0, 'f', 1));
    m_mslLabel    ->setText(QString("%1 m").arg(mslM, 0, 'f', 1));
    m_sivLabel    ->setText(QString::number(EMUL_GPS_SIV));
    m_pdopLabel   ->setText(QString("%1").arg(EMUL_GPS_PDOP / 10.0, 0, 'f', 2));
    m_speedLabel  ->setText(QString("%1 m/s  (Vz %2  Vh %3)")
        .arg(totalSpeed, 0, 'f', 1).arg(vz_ms, 0, 'f', 1).arg(m_vehicle->vh(), 0, 'f', 1));
    m_ecefLabel   ->setText(QString("%1 / %2 / %3").arg(ecefX).arg(ecefY).arg(ecefZ));
    m_velLabel    ->setText(QString("Vx %1  Vy %2  Vz %3 m/s  (t=%4 s)")
        .arg(vx_ms, 0, 'f', 2).arg(vy_ms, 0, 'f', 2).arg(vz_ms, 0, 'f', 2)
        .arg(m_vehicle->inFlight(), 0, 'f', 1));
    m_packetLabel ->setText(QString::number(m_packetCount));
    m_fixLabel    ->setText(EMUL_GPS_FIX < 5 ? fixNames[EMUL_GPS_FIX] : QString::number(EMUL_GPS_FIX));
    m_rtkLabel    ->setText(EMUL_GPS_RTK < 3 ? rtkNames[EMUL_GPS_RTK] : QString::number(EMUL_GPS_RTK));

    QString displayLine = line;
    displayLine.remove('\r');
    if (displayLine.length() > EMUL_LAST_LINE_MAX_CHARS)
        displayLine = displayLine.left(EMUL_LAST_LINE_MAX_CHARS - 3) + "...";
    m_lastLineLabel->setText(displayLine);
}
