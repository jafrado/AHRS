#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QTcpServer>
#include <QTcpSocket>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QCheckBox>

#include "vehicle_rocket.h"

class EmulatorWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit EmulatorWindow(QWidget *parent = nullptr);
    ~EmulatorWindow();

private slots:
    void onStartStop();
    void onTick();
    void onNewConnection();
    void onClientDisconnected();
    void onMotorChanged(const QString &text);

private:
    void buildUi();
    void setRunning(bool running);
    void updateMotorDerived();       // parse motor text → setParams/setup on vehicle, update label

    // Network controls
    QSpinBox        *m_tcpPortSpinBox;
    QSpinBox        *m_rateSpinBox;
    QPushButton     *m_startStopBtn;
    QLabel          *m_statusLabel;

    // Rocket parameter widgets
    QLineEdit       *m_motorEdit;             // e.g. "N1000"
    QDoubleSpinBox  *m_loadedWtSpinBox;       // lbs
    QDoubleSpinBox  *m_propWtSpinBox;         // lbs
    QDoubleSpinBox  *m_launchAngleSpinBox;    // degrees from vertical
    QSpinBox        *m_launchAltSpinBox;      // feet MSL
    QDoubleSpinBox  *m_cdSpinBox;             // pressure drag coefficient (Cd)
    QDoubleSpinBox  *m_cfSpinBox;             // skin friction coefficient (Cf)
    QDoubleSpinBox  *m_diameterSpinBox;       // body diameter, inches
    QDoubleSpinBox  *m_lengthSpinBox;         // body length, feet
    QLabel          *m_derivedLabel;          // shows burn time, Ve, Isp, dry mass

    // Parachute controls
    QCheckBox       *m_drogueCheckBox;        // deploy drogue at apogee (30 ft/s)
    QCheckBox       *m_mainChuteCheckBox;     // deploy main chute at specified altitude (5 ft/s)
    QSpinBox        *m_mainChuteAltSpinBox;   // main chute deploy altitude (ft AGL)
    QLabel          *m_mainChuteAltLabel;     // "Deploy Alt (ft):" label

    // Live value labels
    QLabel *m_phaseLabel;
    QLabel *m_yawLabel;
    QLabel *m_pitchLabel;
    QLabel *m_rollLabel;
    QLabel *m_headingLabel;
    QLabel *m_latLabel;
    QLabel *m_lonLabel;
    QLabel *m_altLabel;
    QLabel *m_mslLabel;
    QLabel *m_sivLabel;
    QLabel *m_fixLabel;
    QLabel *m_rtkLabel;
    QLabel *m_pdopLabel;
    QLabel *m_speedLabel;
    QLabel *m_ecefLabel;
    QLabel *m_velLabel;
    QLabel *m_packetLabel;
    QLabel *m_lastLineLabel;

    // Network state
    QTimer             *m_timer;
    QTcpServer         *m_server;
    QList<QTcpSocket*>  m_clients;
    bool                m_running;
    qint64              m_packetCount;

    // Vehicle — owns all flight physics and state
    VehicleRocket      *m_vehicle;
};
