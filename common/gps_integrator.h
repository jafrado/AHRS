#pragma once
#include <cmath>

// GPSIntegrator — WGS84 dead-reckoning position integrator.
//
// Shared between the AHRS receiver (Qt app) and the Vehicle emulator.
// Pure C++ — no Qt dependency.
//
// Usage:
//   seed()      — set initial position from a known GPS fix.
//   integrate() — advance position one tick using NED velocity.
//
// NED convention: Vn = north m/s, Ve = east m/s, altMslM = altitude MSL (m).
// Vertical velocity is NOT integrated; altitude is passed in directly each tick
// from an authoritative source (physics integrator or sensor altitude field).

class GPSIntegrator {
public:
    void seed(double latDeg, double lonDeg, double altMslM);
    void integrate(double vnMs, double veMs, double altMslM, double dt);

    bool   isSeeded() const { return m_seeded; }
    double latDeg()   const { return m_latDeg; }
    double lonDeg()   const { return m_lonDeg; }
    double altMslM()  const { return m_altMslM; }
    double ecefX()    const { return m_ecefX; }
    double ecefY()    const { return m_ecefY; }
    double ecefZ()    const { return m_ecefZ; }

    // WGS84 coordinate conversions (public — available to callers).
    static void llaToECEF(double latDeg, double lonDeg, double altM,
                           double &X, double &Y, double &Z);
    static void ecefToLLA(double X, double Y, double Z,
                           double &latDeg, double &lonDeg, double &altM);

private:
    bool   m_seeded  = false;
    double m_latDeg  = 0.0;
    double m_lonDeg  = 0.0;
    double m_altMslM = 0.0;
    double m_ecefX   = 0.0;
    double m_ecefY   = 0.0;
    double m_ecefZ   = 0.0;
};
