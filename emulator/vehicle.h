#pragma once

#include <QObject>
#include <QString>
#include "../common/gps_integrator.h"

// Abstract base class for simulated flight vehicles.
//
// The simulator loop in EmulatorWindow calls the API methods each tick:
//   setup()  — derive internal parameters from the current params struct;
//               call once when the server starts (before init).
//   init()   — reset all flight state to pre-launch; call after setup().
//   run(dt)  — advance the simulation by dt seconds; called every timer tick.
//   launch() — transition from the waiting state to active flight;
//               called when the first TCP client connects.
//
// After each run() call the emulator window reads telemetry via the public
// getters and formats the $INS packet.

class Vehicle : public QObject
{
    Q_OBJECT

public:
    explicit Vehicle(QObject *parent = nullptr);
    virtual ~Vehicle() = default;

    // ── Simulator API ────────────────────────────────────────────────────────
    virtual void setup()        = 0;   // Derive parameters; call before init()
    virtual void init()         = 0;   // Reset to pre-launch state
    virtual void run(double dt) = 0;   // Advance one simulation tick (dt in seconds)
    virtual void launch()       = 0;   // Trigger launch (first TCP client connection)

    // ── Attitude / velocity telemetry ────────────────────────────────────────
    double  yaw()       const { return m_yaw; }
    double  pitch()     const { return m_pitch; }
    double  roll()      const { return m_roll; }
    double  altFt()     const { return m_altFt; }
    double  vz()        const { return m_vz; }     // vertical speed m/s  (up = +)
    double  vh()        const { return m_vh; }     // horizontal speed m/s
    double  vxMs()      const { return m_vxMs; }   // world-frame north velocity m/s
    double  vyMs()      const { return m_vyMs; }   // world-frame east  velocity m/s
    double  inFlight()  const { return m_inFlight; }
    bool    isLanded()  const { return m_isLanded; }
    const QString &phaseName() const { return m_phaseName; }

    // ── GPS / position telemetry ─────────────────────────────────────────────
    double  latDeg()    const { return m_gpsInt.latDeg(); }   // geodetic latitude  (degrees)
    double  lonDeg()    const { return m_gpsInt.lonDeg(); }   // geodetic longitude (degrees)
    double  altMslM()   const { return m_gpsInt.altMslM(); }  // altitude MSL (metres)
    double  ecefX()     const { return m_gpsInt.ecefX(); }    // ECEF X (metres)
    double  ecefY()     const { return m_gpsInt.ecefY(); }    // ECEF Y (metres)
    double  ecefZ()     const { return m_gpsInt.ecefZ(); }    // ECEF Z (metres)

protected:
    // ── Attitude / velocity state ─────────────────────────────────────────────
    double  m_yaw      = 0.0;
    double  m_pitch    = 0.0;
    double  m_roll     = 0.0;
    double  m_altFt    = 0.0;   // altitude MSL (feet) — authoritative from physics
    double  m_vz       = 0.0;
    double  m_vh       = 0.0;
    double  m_vxMs     = 0.0;
    double  m_vyMs     = 0.0;
    double  m_inFlight = 0.0;
    bool    m_isLanded = false;
    QString m_phaseName{ "PRELAUNCH" };

    // ── GPS / position state ──────────────────────────────────────────────────
    GPSIntegrator m_gpsInt;
};
