#pragma once

#include "vehicle.h"

// All parameters supplied by the emulator UI.
// EmulatorWindow builds this struct from its spinboxes and passes it to
// VehicleRocket::setParams() before calling setup() and init().
struct RocketParams {
    // Motor (derived from TRA designation string)
    double totalImpulseNs = 20480.0;    // Ns   — N class
    double avgThrustN     = 1000.0;     // N    — N1000

    // Mass (kg — converted from UI lbs / grams)
    double loadedMassKg   = 18.214;     // airframe (5.443) + N1000W loaded (12.771)
    double propMassKg     = 8.293;      // N1000W propellant

    // Launch geometry
    double launchAngleDeg = 10.0;       // degrees from vertical
    double launchAltFt    = 4000.0;     // ft MSL — Blackrock Desert, NV
    double launchLatDeg   = 40.88325;   // N40° 52' 59.7"  Blackrock Desert
    double launchLonDeg   = -119.034639;// W119° 2' 4.7"   Blackrock Desert

    // Aerodynamics (two-term drag model)
    double cd             = 0.75;       // pressure drag coefficient (frontal area)
    double cf             = 0.004;      // skin friction coefficient (lateral area)
    double diameterIn     = 4.0;        // body tube outer diameter, inches
    double lengthFt       = 6.2;        // airframe length, feet (6 ft 2 in)

    // Recovery
    bool   drogueChute    = true;
    bool   mainChute      = true;
    double mainChuteAglFt = 1000.0;     // AGL altitude for main chute deployment
};

// Six-phase rocket flight vehicle.
//
// Physics model:
//   POWERED  — Newton's second law with time-varying mass (propellant consumed at mdot)
//   COAST    — ballistic: gravity + two-term aerodynamic drag on dry mass
//   APOGEE   — 2-second parachute deployment window; physics continues, rocket arcs over
//   DESCENT  — drogue (30 ft/s), main chute (5 ft/s), or ballistic (body drag only)
//   LANDED   — decaying tumble oscillation
//
// Drag is split into pressure drag (Cd × A_front) and skin friction (Cf × A_side):
//   Fd = 0.5·ρ·Cd·π(d/2)²·V²  +  0.5·ρ·Cf·π·d·L·V²
//
// Atmosphere: ρ = 1.22 × 0.9^(alt_m / 1000)  (exponential model, ~14 km valid)

class VehicleRocket : public Vehicle
{
    Q_OBJECT

public:
    explicit VehicleRocket(QObject *parent = nullptr);

    void setParams(const RocketParams &p);

    // Simulator API
    void setup()        override;   // Derive burnTime, dryMass, mdot, Ve, Isp from params
    void init()         override;   // Reset to PRELAUNCH at launch site altitude
    void run(double dt) override;   // Advance one simulation tick
    void launch()       override;   // Transition PRELAUNCH → POWERED

    // Derived motor parameters — read by EmulatorWindow for the derived info label
    double burnTime()   const { return m_burnTime; }
    double ve()         const { return m_ve; }
    double isp()        const { return m_isp; }
    double dryMassKg()  const { return m_dryMassKg; }

    // TRA total impulse lookup — public so EmulatorWindow can validate motor input
    static double traTotalImpulse(QChar letter);

private:
    enum Phase { PRELAUNCH, POWERED, COAST, APOGEE, DESCENT, LANDED };
    static const char  *s_phaseNames[];

    RocketParams m_p;

    Phase  m_phase      = PRELAUNCH;
    double m_tPhase     = 0.0;         // time within current phase (s)
    double m_launchYaw  = 0.0;         // heading at launch (deg), fixed = 0 (north)
    double m_landingYaw = 0.0;         // yaw captured at touchdown

    // Derived in setup()
    double m_burnTime   = 0.0;
    double m_dryMassKg  = 0.0;
    double m_mdot       = 0.0;
    double m_ve         = 0.0;
    double m_isp        = 0.0;

    void integratePhysics(double dt);
    void computeAttitude();
};
