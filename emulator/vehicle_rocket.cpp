#include "vehicle_rocket.h"

#include <cmath>
#include <QtMath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ── Physical constants ────────────────────────────────────────────────────────

// Standard acceleration of gravity (m/s²).
// ISO 80000-3 defined value; also adopted by NIST SP330.
// Used in every force/acceleration calculation (thrust, drag, parachute drag, weight).
#define GRAVITY_MS2             9.80665

// Unit conversion — 1 international foot = 0.3048 m exactly.
// Defined by the International Yard and Pound Agreement (1959); adopted in NIST Handbook 44.
// Used to convert altitude (ft → m) and airframe length (ft → m).
#define FT_TO_M                 0.3048

// Unit conversion — 1 m = 1/0.3048 ≈ 3.28084 ft.
// Reciprocal of FT_TO_M.  The physics integrator keeps altitude in feet to match
// the UI; vertical velocity is in m/s, so this factor converts it to ft/s for
// the altitude update: Δalt_ft = Vz_ms × M_TO_FT × dt
#define M_TO_FT                 3.28084

// Unit conversion — 1 international inch = 0.0254 m exactly.
// Derived from FT_TO_M / 12; per NIST Handbook 44.
// Used to convert body-tube diameter from UI inches to metres for area calculations.
#define IN_TO_M                 0.0254

// ── Atmosphere model ──────────────────────────────────────────────────────────

// Sea-level air density (kg/m³).
// ICAO International Standard Atmosphere exact value is 1.225 kg/m³; 1.22 is used
// here as a round-number approximation consistent with the exponential model below.
#define RHO_SEA_LEVEL_KGM3      1.22

// Exponential atmosphere density scale factor per 1 000 m altitude.
// ρ(h) = RHO_SEA_LEVEL_KGM3 × ATM_DENSITY_SCALE^(h/1000)
// Gives ~10 % density decrease per kilometre, matching the ISA standard atmosphere
// to within ~5 % below 14 km.  Derived from the barometric formula with a 6.5 K/km
// lapse rate; the 0.9 factor is an empirical fit to the lower troposphere.
#define ATM_DENSITY_SCALE       0.9

// ── Minimum dry mass ──────────────────────────────────────────────────────────

// Minimum allowable dry mass (kg) — guards against division by zero if the user
// enters propellant mass ≥ loaded mass.  0.01 kg = 10 g, far below any real airframe.
#define MIN_DRY_MASS_KG         0.01

// ── Parachute terminal velocities ─────────────────────────────────────────────

// Drogue chute terminal descent velocity (ft/s).
// 100 ft/s ≈ 30.5 m/s — high-speed drogue for minimum drift before main deploy.
#define DROGUE_TERM_FPS         100.0

// Main chute terminal descent velocity (ft/s).
// 5 ft/s ≈ 1.52 m/s is a common sport-rocketry main-chute target; slow enough
// for a safe touchdown without excessive horizontal drift.
#define MAIN_TERM_FPS           5.0

// ── Apogee deployment window ──────────────────────────────────────────────────

// Duration of the simulated apogee / parachute-deployment phase (seconds).
// A real black-powder ejection charge fires after a 1–3 s delay from motor burnout.
// 2 s allows the physics to naturally arc the rocket over before DESCENT begins.
#define APOGEE_DEPLOY_S         2.0

// ── Speed thresholds ──────────────────────────────────────────────────────────

// Minimum total airspeed (m/s) required to decompose drag along the velocity vector.
// Below this threshold the velocity direction is numerically undefined; drag is skipped.
#define MIN_SPEED_DRAG_MS       0.001

// Minimum total airspeed (m/s) for reliable velocity-vector attitude tracking.
// Below this the velocity vector direction is nearly arbitrary; attitude falls back
// to a fixed pose (launch angle on the pad, or nose-down under chute).
#define MIN_SPEED_ATTITUDE_MS   0.1

// ── Under-canopy attitude animation parameters ────────────────────────────────

// Nose-down pitch baseline while hanging under the canopy (degrees).
// A real rocket hangs nearly vertically nose-down (≈ −90°); −75° gives a visible
// tilt in the attitude indicator without obscuring the horizon line.
#define CANOPY_PITCH_BASELINE_DEG   -75.0

// Pitch pendulum oscillation amplitude (degrees).
// Sized so the tip of the nose swings ±20° either side of the baseline,
// producing a visually plausible pendulum arc for a 10–15 m suspension line.
#define CANOPY_PITCH_AMP_DEG         20.0

// Pitch pendulum oscillation frequency (rad/s).
// Period ≈ 2π/0.8 ≈ 7.9 s.  For a simple pendulum, L = g/(ω²) ≈ 15 m,
// consistent with a sport-rocketry shock-cord length.
#define CANOPY_PITCH_FREQ_RADS        0.8

// Roll oscillation amplitude (degrees).
// Smaller than pitch to suggest a coupled, slightly asymmetric pendulum swing.
#define CANOPY_ROLL_AMP_DEG          12.0

// Roll oscillation frequency (rad/s); slightly lower than pitch to give a
// non-repeating Lissajous-like coupled motion.
#define CANOPY_ROLL_FREQ_RADS         0.55

// Roll phase offset (radians).  Breaks the symmetry between pitch and roll so
// the oscillation is not confined to a single plane, producing a figure-8 trace.
#define CANOPY_ROLL_PHASE_RAD         1.3

// Slow yaw spin rate under the canopy (degrees/second).
// 18 deg/s → one full rotation every 20 s, typical for a lightly asymmetric canopy.
#define CANOPY_SPIN_DEGPS            18.0

// ── Landed tumble animation parameters ────────────────────────────────────────

// Initial pitch amplitude immediately after touchdown (degrees).
// Represents the rocket bouncing or tipping over on the ground.
#define LANDED_PITCH_AMP_DEG         30.0

// Initial roll amplitude immediately after touchdown (degrees).
#define LANDED_ROLL_AMP_DEG          25.0

// Pitch tumble oscillation frequency (rad/s).
#define LANDED_PITCH_FREQ_RADS        3.0

// Roll tumble oscillation frequency (rad/s); offset from pitch to avoid lock-step.
#define LANDED_ROLL_FREQ_RADS         2.2

// Roll phase offset (radians) — desynchronises pitch and roll tumble axes.
#define LANDED_ROLL_PHASE_RAD         0.9

// Exponential tumble decay rate (1/s).
// amplitude × e^(−LANDED_DECAY_RATE × t) → effectively zero by ~10 s:
//   e^(−0.35 × 10) ≈ 0.03, i.e. 3 % of initial amplitude remains after 10 s.
#define LANDED_DECAY_RATE             0.35

// ─────────────────────────────────────────────────────────────────────────────

const char *VehicleRocket::s_phaseNames[] = {
    "PRELAUNCH", "POWERED", "COAST", "APOGEE", "DESCENT", "LANDED"
};

// ── TRA total impulse table ───────────────────────────────────────────────────
// TRA/NAR motor designation: the letter identifies the impulse class.
// Each class has exactly twice the total impulse of the previous class,
// starting at 2.5 Ns for class A (0–2.5 Ns range → 2.5 Ns upper bound).
// Source: Tripoli Rocketry Association motor classification standard.

double VehicleRocket::traTotalImpulse(QChar letter)
{
    switch (letter.toUpper().toLatin1()) {
    case 'A': return 2.5;       // A:    0 – 2.5 Ns
    case 'B': return 5.0;       // B:  2.5 – 5   Ns  (2× A)
    case 'C': return 10.0;      // C:    5 – 10  Ns  (2× B)
    case 'D': return 20.0;      // D:   10 – 20  Ns  (2× C)
    case 'E': return 40.0;      // E:   20 – 40  Ns  (2× D)
    case 'F': return 80.0;      // F:   40 – 80  Ns  (2× E)
    case 'G': return 160.0;     // G:   80 – 160 Ns  (2× F)
    case 'H': return 320.0;     // H:  160 – 320 Ns  (2× G)
    case 'I': return 640.0;     // I:  320 – 640 Ns  (2× H)
    case 'J': return 1280.0;    // J:  640 – 1280 Ns (2× I)
    case 'K': return 2560.0;    // K: 1280 – 2560 Ns (2× J)
    case 'L': return 5120.0;    // L: 2560 – 5120 Ns (2× K)
    case 'M': return 10240.0;   // M:  5120 – 10240 Ns (2× L)
    case 'N': return 20480.0;   // N: 10240 – 20480 Ns (2× M)
    case 'O': return 40960.0;   // O: 20480 – 40960 Ns (2× N)
    default:  return 0.0;
    }
}

// ── Constructor ───────────────────────────────────────────────────────────────

VehicleRocket::VehicleRocket(QObject *parent)
    : Vehicle(parent)
{
    setup();
    init();
}

// ── Simulator API ─────────────────────────────────────────────────────────────

void VehicleRocket::setParams(const RocketParams &p)
{
    m_p = p;
}

void VehicleRocket::setup()
{
    // Burn time from the definition of average thrust: F_avg = I / t_burn → t_burn = I / F_avg
    m_burnTime  = (m_p.avgThrustN > 0.0) ? m_p.totalImpulseNs / m_p.avgThrustN : 0.0;

    // Dry mass = loaded launch mass − propellant mass consumed during burn.
    // Clamped to MIN_DRY_MASS_KG to prevent division by zero in drag/acceleration terms.
    m_dryMassKg = qMax(MIN_DRY_MASS_KG, m_p.loadedMassKg - m_p.propMassKg);

    // Propellant mass flow rate (kg/s): ṁ = m_prop / t_burn.
    // Assumes constant thrust (average) and uniform propellant consumption over the burn.
    m_mdot = (m_burnTime > 0.0) ? m_p.propMassKg / m_burnTime : 0.0;

    // Effective exhaust velocity (m/s): derived from total impulse = m_prop × Ve → Ve = I / m_prop.
    // Ve is the single constant exhaust velocity that produces the stated total impulse
    // when m_prop kg of propellant is expelled.
    m_ve = (m_p.propMassKg > 0.0) ? m_p.totalImpulseNs / m_p.propMassKg : 0.0;

    // Specific impulse (seconds): Isp = Ve / g₀.
    // Motor efficiency metric; higher Isp = more impulse per unit propellant weight.
    m_isp = m_ve / GRAVITY_MS2;
}

void VehicleRocket::init()
{
    m_phase      = PRELAUNCH;
    m_tPhase     = 0.0;
    m_inFlight   = 0.0;
    m_altFt      = m_p.launchAltFt;
    m_vz         = 0.0;
    m_vh         = 0.0;
    m_vxMs       = 0.0;
    m_vyMs       = 0.0;
    m_launchYaw  = 0.0;
    m_landingYaw = 0.0;
    m_isLanded   = false;
    m_phaseName  = s_phaseNames[PRELAUNCH];

    // Static attitude on the pad: pitch = 90° − launch_angle gives the angle above
    // horizontal (a vertical rail is 0° from vertical → 90° pitch; a 10° tilt → 80° pitch).
    m_pitch = 90.0 - m_p.launchAngleDeg;
    m_roll  = 0.0;
    m_yaw   = m_launchYaw;

    // Seed the GPS integrator at the launch site; converts (lat, lon, alt) to ECEF internally.
    m_gpsInt.seed(m_p.launchLatDeg, m_p.launchLonDeg, m_p.launchAltFt * FT_TO_M);
}

void VehicleRocket::launch()
{
    if (m_phase != PRELAUNCH) return;
    m_phase    = POWERED;
    m_inFlight = 0.0;
    m_tPhase   = 0.0;
    m_launchYaw = 0.0;   // fixed launch heading: north (0°)
    m_phaseName = s_phaseNames[POWERED];
}

// ── Simulation tick ───────────────────────────────────────────────────────────

void VehicleRocket::run(double dt)
{
    integratePhysics(dt);
    computeAttitude();

    // Decompose horizontal speed into world-frame NED components using the current yaw.
    // m_vh is the scalar horizontal speed (m/s); yaw is the compass bearing (degrees).
    double yawRad = m_yaw * M_PI / 180.0;
    m_vxMs = m_vh * sin(yawRad);   // north velocity component (m/s): Vn = Vh × sin(yaw)
    m_vyMs = m_vh * cos(yawRad);   // east  velocity component (m/s): Ve = Vh × cos(yaw)
    // m_vz is already the vertical speed in m/s (up = positive); no conversion needed

    // Integrate GPS position using NED velocity and authoritative physics altitude.
    // The altitude passed is the current physics altitude in metres; the integrator
    // does not integrate Vz independently — altitude comes from the physics loop.
    m_gpsInt.integrate(m_vxMs, m_vyMs, m_altFt * FT_TO_M, dt);

    m_phaseName = s_phaseNames[m_phase];
    m_isLanded  = (m_phase == LANDED);
}

// ── Physics integration ───────────────────────────────────────────────────────

void VehicleRocket::integratePhysics(double dt)
{
    const double launchAltFt = m_p.launchAltFt;

    // Launch angle in radians for thrust decomposition trig
    const double angleRad = m_p.launchAngleDeg * M_PI / 180.0;

    // ── Aerodynamic drag (two-term model) ─────────────────────────────────────
    // Pressure drag acts on the frontal disc; skin friction acts on the lateral
    // cylindrical surface.  Using a single Cd over both areas would overestimate
    // friction drag by 1–2 orders of magnitude on a slender rocket body.

    // Altitude in metres — needed for the atmosphere model (physics loop uses feet)
    double altM = m_altFt * FT_TO_M;

    // Air density via exponential model: ρ(h) = ρ₀ × 0.9^(h/1000)
    // Gives ≈ 10 % decrease per km; good to ~5 % error below 14 km altitude
    double rho = RHO_SEA_LEVEL_KGM3 * pow(ATM_DENSITY_SCALE, altM / 1000.0);

    // Convert body dimensions from UI units to SI for area calculations
    double diamM   = m_p.diameterIn * IN_TO_M;   // outer diameter (m)
    double lengthM = m_p.lengthFt   * FT_TO_M;   // airframe length (m)

    // Frontal (pressure) reference area: circular cross-section, A = π(d/2)²
    double A_front = M_PI * (diamM / 2.0) * (diamM / 2.0);

    // Lateral (friction) reference area: open cylinder surface, A = π·d·L
    double A_side  = M_PI * diamM * lengthM;

    // Total airspeed (m/s) — scalar speed magnitude driving all drag terms
    double V_total = sqrt(m_vz * m_vz + m_vh * m_vh);

    // Pressure drag force: Fd = ½ρ·Cd·A_front·V²  (N)
    // Acts on the frontal disc; Cd ≈ 0.75 for a typical body-tube nose-cone combination
    double Fd_pressure = 0.5 * rho * m_p.cd * A_front * V_total * V_total;

    // Skin friction drag force: Fd = ½ρ·Cf·A_side·V²  (N)
    // Acts on the lateral cylindrical surface; Cf ≈ 0.004 for a painted finish
    double Fd_friction = 0.5 * rho * m_p.cf * A_side  * V_total * V_total;

    // Total aerodynamic drag magnitude (N)
    double Fd = Fd_pressure + Fd_friction;

    // Decompose drag along the velocity vector so it correctly opposes both
    // vertical and horizontal motion: F_component = −Fd × V_component / |V|
    double Fd_z = 0.0, Fd_h = 0.0;
    if (V_total > MIN_SPEED_DRAG_MS) {
        Fd_z = -Fd * m_vz / V_total;   // vertical drag (N), negative → opposes upward speed
        Fd_h = -Fd * m_vh / V_total;   // horizontal drag (N), negative → opposes forward speed
    }

    // ── Phase integration ─────────────────────────────────────────────────────

    if (m_phase == POWERED) {
        m_inFlight += dt;
        m_tPhase   += dt;

        // Propellant mass burned so far: Δm = ṁ × t; clamped to available propellant
        double massBurned  = qMin(m_mdot * m_inFlight, m_p.propMassKg);

        // Current rocket mass: m(t) = m₀ − Δm; clamped at dry mass once propellant is exhausted
        double currentMass = qMax(m_p.loadedMassKg - massBurned, m_dryMassKg);

        // Decompose average thrust along the fixed launch angle (angle measured from vertical):
        //   vertical component   = F × cos(angle from vertical)
        //   horizontal component = F × sin(angle from vertical)
        double thrustZ = m_p.avgThrustN * cos(angleRad);
        double thrustH = m_p.avgThrustN * sin(angleRad);

        // Euler integration of Newton's second law, vertical axis:
        //   a_z = (F_thrust_z + F_drag_z) / m − g
        // Drag is already negative when opposing upward motion; gravity subtracts directly.
        m_vz += ((thrustZ + Fd_z) / currentMass - GRAVITY_MS2) * dt;

        // Horizontal axis: no gravity term (gravity acts perpendicular to horizontal plane)
        //   a_h = (F_thrust_h + F_drag_h) / m
        m_vh += ((thrustH + Fd_h) / currentMass) * dt;
        if (m_vh < 0.0) m_vh = 0.0;   // m_vh is a speed magnitude; cannot go negative

        // Update altitude: Vz (m/s) × M_TO_FT (ft/m) × dt (s) → Δalt (ft)
        m_altFt += m_vz * M_TO_FT * dt;
        if (m_altFt < launchAltFt) m_altFt = launchAltFt;   // floor at launch-pad elevation

        // Transition to COAST when all propellant has been consumed
        if (m_inFlight >= m_burnTime) {
            m_phase  = COAST;
            m_tPhase = 0.0;
        }

    } else if (m_phase == COAST) {
        m_inFlight += dt;
        m_tPhase   += dt;

        // Coasting on dry mass: only aerodynamic drag and gravity act on the vehicle.
        // a_z = F_drag_z / m_dry − g
        m_vz += (Fd_z / m_dryMassKg - GRAVITY_MS2) * dt;

        // Horizontal: only drag (gravity perpendicular to horizontal plane)
        // a_h = F_drag_h / m_dry
        m_vh += (Fd_h / m_dryMassKg) * dt;
        if (m_vh < 0.0) m_vh = 0.0;

        m_altFt += m_vz * M_TO_FT * dt;
        if (m_altFt < launchAltFt) m_altFt = launchAltFt;

        // Natural apogee detected when vertical velocity crosses zero
        if (m_vz <= 0.0) {
            // Carry all velocity state through unchanged so the rocket arcs naturally
            m_phase  = APOGEE;
            m_tPhase = 0.0;
        }

    } else if (m_phase == APOGEE) {
        // Physics continues during the parachute deployment window so the rocket
        // arcs over naturally rather than snapping to a fixed nose-down attitude.
        m_tPhase += dt;

        // Same ballistic equations as COAST: drag + gravity on dry mass
        m_vz += (Fd_z / m_dryMassKg - GRAVITY_MS2) * dt;
        m_vh += (Fd_h / m_dryMassKg) * dt;
        if (m_vh < 0.0) m_vh = 0.0;

        m_altFt += m_vz * M_TO_FT * dt;
        if (m_altFt < launchAltFt) m_altFt = launchAltFt;

        // Transition to DESCENT after the ejection-charge delay window
        if (m_tPhase >= APOGEE_DEPLOY_S) {
            m_phase  = DESCENT;
            m_tPhase = 0.0;
        }

    } else if (m_phase == DESCENT) {
        m_tPhase += dt;

        // Determine which recovery device is active based on altitude
        const double mainAltFt  = m_p.launchAltFt + m_p.mainChuteAglFt;
        const bool   mainActive   = m_p.mainChute   && (m_altFt <= mainAltFt);
        const bool   drogueActive = m_p.drogueChute && !mainActive;

        if (mainActive) {
            // Main chute: terminal velocity MAIN_TERM_FPS ft/s converted to m/s
            const double V_term = MAIN_TERM_FPS * FT_TO_M;

            // Parachute drag coefficient sized so F_drag = m·g at terminal velocity:
            //   At V_term:  m·g = k·V_term²  →  k = m·g / V_term²
            const double chute_k = m_dryMassKg * GRAVITY_MS2 / (V_term * V_term);

            // Net vertical force: gravity (down) + drag (always opposing motion).
            //   F_drag = −k · Vz · |Vz|  — negative when falling (Vz<0) → upward drag;
            //                              positive when rising  (Vz>0) → downward drag.
            //   Using m_vz*m_vz (always positive) instead would flip drag direction on any
            //   upward overshoot and cause exponential divergence to ±Inf.
            double F_net = -m_dryMassKg * GRAVITY_MS2 - chute_k * m_vz * fabs(m_vz);

            // a = F_net / m; integrate Vz
            m_vz += (F_net / m_dryMassKg) * dt;

            // Clamp: cannot fall faster than terminal, and parachute prevents upward motion
            if (m_vz < -V_term) m_vz = -V_term;
            if (m_vz >  0.0)    m_vz =  0.0;

        } else if (drogueActive) {
            // Drogue chute: same direction-aware drag model, higher terminal velocity
            const double V_term  = DROGUE_TERM_FPS * FT_TO_M;
            const double chute_k = m_dryMassKg * GRAVITY_MS2 / (V_term * V_term);
            double F_net = -m_dryMassKg * GRAVITY_MS2 - chute_k * m_vz * fabs(m_vz);
            m_vz += (F_net / m_dryMassKg) * dt;
            if (m_vz < -V_term) m_vz = -V_term;
            if (m_vz >  0.0)    m_vz =  0.0;

        } else {
            // Ballistic descent: body aerodynamic drag only (no parachute)
            // Same equations as COAST — gravity + body drag on dry mass
            m_vz += (Fd_z / m_dryMassKg - GRAVITY_MS2) * dt;
            m_vh += (Fd_h / m_dryMassKg) * dt;
            if (m_vh < 0.0) m_vh = 0.0;
        }

        m_altFt += m_vz * M_TO_FT * dt;

        // Touchdown: altitude has returned to launch-pad elevation
        if (m_altFt <= launchAltFt) {
            m_altFt      = launchAltFt;   // snap to ground — prevent sub-surface values
            m_vz         = 0.0;
            m_vh         = 0.0;
            // Landing yaw: accumulated canopy spin since DESCENT began
            // (CANOPY_SPIN_DEGPS deg/s × elapsed time, offset from launch heading)
            m_landingYaw = fmod(m_launchYaw + m_tPhase * CANOPY_SPIN_DEGPS, 360.0);
            m_phase      = LANDED;
            m_tPhase     = 0.0;
        }

    } else if (m_phase == LANDED) {
        m_tPhase += dt;
    }
    // PRELAUNCH: no integration — rocket is stationary on the pad
}

// ── Attitude ──────────────────────────────────────────────────────────────────

void VehicleRocket::computeAttitude()
{
    switch (m_phase) {

    case PRELAUNCH:
        // Rocket is vertical on the rail, pitched back by the launch angle.
        // pitch = 90° − launch_angle: a 10° tilt from vertical gives pitch = 80°.
        m_pitch = 90.0 - m_p.launchAngleDeg;
        m_roll  = 0.0;
        m_yaw   = m_launchYaw;
        break;

    case POWERED:
    case COAST:
    {
        // Track the velocity vector: pitch = angle of the velocity vector above horizontal.
        // atan2(Vz, Vh) gives the elevation angle in radians; converted to degrees.
        double spd = sqrt(m_vz * m_vz + m_vh * m_vh);
        m_pitch = (spd > MIN_SPEED_ATTITUDE_MS)
                      ? atan2(m_vz, m_vh) * 180.0 / M_PI
                      : 90.0 - m_p.launchAngleDeg;  // fallback: pad attitude if nearly stationary
        m_roll = 0.0;
        m_yaw  = m_launchYaw;
        break;
    }

    case APOGEE:
    {
        // Continue velocity-vector tracking through the arc — same formula as COAST.
        // Pitch will sweep from ~0° through negative values as the rocket tips earthward.
        double spd = sqrt(m_vz * m_vz + m_vh * m_vh);
        m_pitch = (spd > MIN_SPEED_ATTITUDE_MS)
                      ? atan2(m_vz, m_vh) * 180.0 / M_PI
                      : 0.0;   // fallback: horizontal if speed is negligible at exact apogee
        m_roll = 0.0;
        m_yaw  = m_launchYaw;
        break;
    }

    case DESCENT:
    {
        const double mainAltFt = m_p.launchAltFt + m_p.mainChuteAglFt;
        const bool   anyChute  = (m_p.drogueChute && m_altFt > mainAltFt)
                               || (m_p.mainChute   && m_altFt <= mainAltFt);
        if (anyChute) {
            // Nose-down pendulum swing under the canopy.
            // pitch = baseline + amplitude × sin(freq × t)
            //   CANOPY_PITCH_BASELINE_DEG + CANOPY_PITCH_AMP_DEG × sin(CANOPY_PITCH_FREQ_RADS × t)
            m_pitch = CANOPY_PITCH_BASELINE_DEG
                    + CANOPY_PITCH_AMP_DEG * sin(CANOPY_PITCH_FREQ_RADS * m_tPhase);

            // Roll oscillates at a different frequency with a phase offset to produce
            // a non-planar Lissajous-like motion: roll = amp × sin(freq × t + phase)
            m_roll = CANOPY_ROLL_AMP_DEG
                   * sin(CANOPY_ROLL_FREQ_RADS * m_tPhase + CANOPY_ROLL_PHASE_RAD);

            // Slow parachute spin: yaw increases linearly at CANOPY_SPIN_DEGPS deg/s
            m_yaw = fmod(m_launchYaw + m_tPhase * CANOPY_SPIN_DEGPS, 360.0);

        } else {
            // Ballistic descent: nose follows the velocity vector (same as COAST)
            double spd = sqrt(m_vz * m_vz + m_vh * m_vh);
            m_pitch = (spd > MIN_SPEED_ATTITUDE_MS)
                          ? atan2(m_vz, m_vh) * 180.0 / M_PI
                          : -90.0;   // falling nearly straight down; default to nose-down
            m_roll = 0.0;
            m_yaw  = m_launchYaw;
        }
        break;
    }

    case LANDED:
    {
        // Decaying tumble oscillation after touchdown.
        // Envelope: e^(−LANDED_DECAY_RATE × t) — amplitude falls to ~3 % by t = 10 s.
        double decay = exp(-LANDED_DECAY_RATE * m_tPhase);

        // pitch = amplitude × decay × sin(freq × t)
        m_pitch = LANDED_PITCH_AMP_DEG * decay * sin(LANDED_PITCH_FREQ_RADS * m_tPhase);

        // roll = amplitude × decay × sin(freq × t + phase)
        // Phase offset ensures roll and pitch do not oscillate in sync
        m_roll  = LANDED_ROLL_AMP_DEG  * decay * sin(LANDED_ROLL_FREQ_RADS  * m_tPhase
                                                     + LANDED_ROLL_PHASE_RAD);
        m_yaw   = m_landingYaw;
        break;
    }

    default:
        m_pitch = m_roll = 0.0;
        m_yaw   = m_launchYaw;
        break;
    }
}
