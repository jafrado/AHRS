#ifndef __AHRS_H
#define __AHRS_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QLCDNumber>
#include <QLabel>
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QtCore>
#include <QtGui>
#include <QBoxLayout>
#include <QVBoxLayout>
#include <QString>
#include <QThread>
#include <QByteArray>
#include <QLabel>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QInputDialog>
#include <QMessageBox>
#include "ahrs_widgets.h"
#include "MASI.h"
#include "../common/gps_integrator.h"


//container for INS/AHRS data
class AHRSData {

public:
    int seqno;
    double lat, lon;
    double x, y, z;
    double vx, vy, vz;
    double yaw, pitch, roll, heading;
    double ground_speed;
    double pdop;
    double altitude, altitudeMSL;
    int siv, fixType, rtkType;
    int yawOffset, yawPolarity;
    int pitchOffset, pitchPolarity;
    int rollOffset, rollPolarity;

    AHRSData() : lat(0), lon(0), x(0), y(0), z(0), vx(0), vy(0), vz(0), yaw(0), pitch(0), roll(0), heading(0),
        ground_speed(0), pdop(0), altitude(0), altitudeMSL(0), siv(0), fixType(0), rtkType(0),
        yawOffset(0), yawPolarity(1), pitchOffset(0), pitchPolarity(1), rollOffset(0), rollPolarity(1) {}
    AHRSData(const AHRSData& a) :
        lat(a.lat), lon(a.lon), x(a.x), y(a.y), z(a.z), vx(a.vx), vy(a.vy), vz(a.vz), yaw(a.yaw), pitch(a.pitch), roll(a.roll), heading(a.heading),
        ground_speed(a.ground_speed), pdop(a.pdop), altitude(a.altitude), altitudeMSL(a.altitudeMSL), siv(a.siv), fixType(a.fixType), rtkType(a.rtkType),
        yawOffset(a.yawOffset), yawPolarity(a.yawPolarity), pitchOffset(a.pitchOffset), pitchPolarity(a.pitchPolarity), rollOffset(a.rollOffset), rollPolarity(a.rollPolarity) {}
    ~AHRSData() {}
};

namespace Ui {
    class AHRSWindow;
}

class AHRSReceiver;

class AHRSWidget : public QWidget {

    Q_OBJECT

public:
    enum Units { Imperial, Metric };

public slots:
    void updateEstimatedPosition(double lat, double lon, double altMslM);
    void updatePose(double yaw, double pitch, double roll);
    void updatePosition(double lat, double lon, double altitude);
    void updateECEFPosition(double ecefX, double ecefY, double ecefZ);
    void updateAltitude(double altitude, double altitudeMSL);
    void updateGroundSpeed(double ground_speed);
    void updateSpeed(double vx, double vy, double vz);
    void updateHeading(double bearing);
    void updatePrecision(double precision);
    void updateStatus(int siv, int fixType, int rtkType);
    void updateHUD(void);
    void resetFlightData();

public:
    AHRSWidget(QWidget* parent = 0);
    void keyPressEvent(QKeyEvent* event);
    void resizeEvent(QResizeEvent* event) override;
    void setReceiver(AHRSReceiver* r);
    void setUnits(Units u);

    void setAHRSFont(const QFont &font);
    void setAHRSColor(const QColor &color);
    void setStatusFont(const QFont &font);
    void setStatusColor(const QColor &color);

    QFont  ahrsFont()    const { return m_ahrsFont; }
    QColor ahrsColor()   const { return m_ahrsColor; }
    QFont  statusFont()  const { return m_statusFont; }
    QColor statusColor() const { return m_statusColor; }
    Units  units()       const { return m_units; }

    MASI*                  masi;
    AHRSAttitudeIndicator* attitude;
    AHRSCompass*           heading;
    AHRSInfo*              info;
    QString deviceName;
    QLabel* deviceLabel;
    QLabel* m_altLabel;
    QLabel* m_climbLabel;
    AHRSReceiver* receiver;

private:
    Units         m_units;
    double        m_lastAlt;
    bool          m_lastAltValid;
    QElapsedTimer m_climbTimer;
    QFont         m_ahrsFont;
    QColor        m_ahrsColor;
    QFont         m_statusFont;
    QColor        m_statusColor;
    int           m_statusLargePx;
    double        m_windowScale;
    QWidget*      m_altPanel;
    void          rebuildAltPanelStyle();

    // Flight statistics panel
    QWidget      *m_flightPanel;
    QLabel       *m_maxAltValLabel;
    QLabel       *m_maxSpeedValLabel;
    QLabel       *m_maxGValLabel;
    QLabel       *m_tApogeeValLabel;
    void          rebuildFlightPanelStyle();

    // Flight statistics tracking
    double        m_maxAltM;       // max altitude seen (metres)
    double        m_maxSpeedMs;    // max total speed seen (m/s)
    double        m_maxGforce;     // max G-force computed from velocity deltas
    double        m_tApogeeS;      // flight-elapsed seconds when max altitude was set
    bool          m_flightActive;    // true once launch confirmed (2 G for 500 ms)
    QElapsedTimer m_flightTimer;    // wall-clock since launch
    double        m_prevVx, m_prevVy, m_prevVz;
    QElapsedTimer m_gTimer;         // delta-t for per-packet G computation
    QElapsedTimer m_launchGTimer;   // how long G has been continuously >= 2 G
};

class AHRSReceiver : public QObject, QRunnable {

    Q_OBJECT

signals:
    void poseChanged(double yaw, double pitch, double roll);
    void positionChanged(double lat, double lon, double altitude);
    void positionECEFChanged(double ecefX, double ecefY, double ecefZ);
    void altitudeChanged(double altitude, double altitudeMSL);
    void groundSpeedChanged(double ground_speed);
    void speedChanged(double vx, double vy, double vz);
    void headingChanged(double bearing);
    void precisionChanged(double precision);
    void statusChanged(int siv, int fixType, int rtkType);
    void estimatedPositionChanged(double lat, double lon, double altMslM);
    void redrawDisplay(void);
public:
    enum Mode { Serial, TCP };

    int lines;
    QString deviceName;
    Mode mode;
    QString tcpHost;
    quint16 tcpPort;
    bool done;
    AHRSData ahrsData;

    AHRSReceiver(QString deviceName);           // serial port
    AHRSReceiver(QString host, quint16 port);   // TCP
    ~AHRSReceiver();
    void run() override;
    void begin();
    void end();
    void restart();
    AHRSData& getData();
    bool isConnected();
    bool validPosition();

private:
    void runTCP();
    void parseLine(const QByteArray &data, AHRSData &current);
    GPSIntegrator m_gpsInt;
    QElapsedTimer m_intTimer;
    bool          m_intTimerStarted = false;
};




#endif /* !__AHRS_H */
