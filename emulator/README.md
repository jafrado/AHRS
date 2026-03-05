# AHRS INS Emulator

A Qt5 GUI application that simulates a high-power rocket flight and streams `$INS` telemetry over TCP, so the AHRS display can be tested without hardware.

The emulator models rocket flight using the physics underlying the **Tsiolkovsky ideal rocket equation** and a six-phase flight state machine. Defaults are configured for an **Aerotech N1000W** motor in a 12 lb airframe launched from the **Blackrock Desert, Lovelock, NV** (LLC319050).

---

## Physics Model

### Propulsion

The [NASA ideal rocket equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-rocket-equation/) gives the closed-form velocity change for a rocket in free space:

```
Δu = V_eq × ln(MR)
```

where `V_eq = I_sp × g₀` is the effective exhaust velocity and `MR = m_full / m_empty` is the mass ratio.

The emulator does not apply this formula directly. Instead it integrates the **differential form** — Newton's second law with time-varying mass — at each simulation tick:

```
m(t) = m₀ − ṁ·t             time-varying mass as propellant is consumed
a    = F_thrust / m(t) − g₀  net acceleration (thrust minus gravity)
v   += a · dt                 Euler integration
```

This is the underlying ODE that the ideal rocket equation is the analytical solution of (in the gravity-free case). Numerically they are equivalent; integrating tick-by-tick also allows gravity, launch angle, and aerodynamic drag to be applied correctly.

All variables derived from motor input are consistent with the ideal rocket equation:

| Variable | Formula | Code |
|---|---|---|
| Burn time | `t_burn = I / F_avg` | `m_burnTime = impulse / thrust` |
| Mass flow rate | `ṁ = m_p / t_burn` | `m_mdot = m_propMassKg / m_burnTime` |
| Exhaust velocity | `V_eq = I / m_p` | `ve = impulse / m_propMassKg` |
| Specific impulse | `I_sp = V_eq / g₀` | `isp = ve / G0` |
| Instantaneous mass | `m(t) = m₀ − ṁ·t` | `currentMass = m_loadedMassKg − m_mdot × t` |

### Aerodynamic Drag

Drag uses a **two-term model** that separates pressure drag (acting on the frontal cross-section) from skin friction drag (acting on the lateral body surface). This is physically correct — applying a single bluff-body Cd to the full wetted area would grossly overestimate friction drag.

```
Fd_pressure = 0.5 × ρ × Cd × A_front × V²    (pressure / form drag)
Fd_friction = 0.5 × ρ × Cf × A_side  × V²    (skin friction drag)
Fd          = Fd_pressure + Fd_friction
```

| Symbol | Description | Default | Notes |
|---|---|---|---|
| `ρ` | Air density (kg/m³) | — | Exponential atmosphere model (see below) |
| `Cd` | Pressure drag coefficient | 0.75 | Applied to frontal disc area only |
| `Cf` | Skin friction coefficient | 0.004 | Applied to lateral cylinder surface only |
| `A_front` | Frontal disc area (m²) | — | `π × (d/2)²` |
| `A_side` | Lateral cylinder area (m²) | — | `π × d × L` (fins excluded) |
| `V` | Total speed (m/s) | — | `√(Vz² + Vh²)` |

**Surface finish — typical Cf values:**

| Finish | Cf |
|---|---|
| Unfinished (raw fibreglass / cardboard) | 0.008 |
| Smooth (sanded) | 0.005 |
| **Painted (default)** | **0.004** |
| Polished | 0.002 |

> A surface finish dropdown (Unfinished / Smooth / Painted / Polished) will be added in a future update to set Cf automatically. Until then, enter Cf directly.

**Area geometry:**

```
A_front = π × (d/2)²    frontal disc — pressure drag reference area
A_side  = π × d × L     lateral cylinder surface — skin friction reference area
```

**Worked example** (default: 4 in diameter, 6 ft 2 in length):

```
d       = 4 in  = 0.1016 m
L       = 6.2 ft = 1.890 m

A_front = π × (0.0508)² = 0.0081 m²
A_side  = π × 0.1016 × 1.890 = 0.603 m²

At sea level (ρ = 1.22 kg/m³), V = 300 m/s:
  Fd_pressure = 0.5 × 1.22 × 0.75 × 0.0081 × 300² =  334 N
  Fd_friction = 0.5 × 1.22 × 0.004 × 0.603 × 300² =  443 N
  Fd_total    =  777 N   (vs. 17,500 N if Cd were naively applied to full wetted area)
```

The drag force is decomposed along the velocity vector so it correctly opposes both vertical and horizontal components:

```
Fd_z = −Fd × Vz / V
Fd_h = −Fd × Vh / V
```

**Atmospheric density model** (exponential approximation):

```
ρ = 1.22 × 0.9^(Y / 1000)
```

where Y is altitude in metres. This gives ~1.22 kg/m³ at sea level and decreases approximately 10% per 1,000 m, matching the standard atmosphere to ~14 km.

The drag equations are incorporated into Newton's second law at each tick:

```
POWERED:           a = (F_thrust + Fd) / m(t) − g
COAST / APOGEE:    a = Fd / m_dry − g
BALLISTIC DESCENT: a = Fd / m_dry − g   (same model, no parachute)
```

Parachute descent uses a separate drag model scaled to the chosen terminal velocity (see DESCENT phase).

**Assumptions and simplifications:**

- Thrust is constant over the burn (average thrust from motor designation)
- Launch angle is fixed throughout powered flight (no gravity turn)
- Cd and Cf are constant (no Mach-number dependence)
- Fin surface area is excluded from the drag calculation
- Parachute drag is modelled as a quadratic drag force calibrated to a fixed terminal velocity; actual chute size and shape are not inputs

---

## Build

Open `AHRSEmulator.pro` in Qt Creator and build, **or** run the batch script:

```bat
cd C:\src\AHRS\emulator
build_emulator.bat
```

---

## Usage

1. Launch the **AHRS INS Emulator**
2. Fill in the **Rocket Parameters** group (see below)
3. Configure the **Parachute** options (drogue, main chute, deploy altitude)
4. Set **TCP Port** and **Rate (ms)** as desired
5. Click **Start Server** — the button turns red and the server begins listening
6. Connect the AHRS app (**Menu → Connect → TCP…**) to `localhost` on the chosen port
7. The simulation launches automatically on the first client connection

All parameter inputs are locked once the server starts. Stop and restart the server to change them.

---

## Rocket Parameters

| Field | Default | Description |
|---|---|---|
| **Motor** | `N1000` | TRA motor designation: letter = impulse class, number = average thrust (N) |
| **Loaded Wt (lbs)** | `40.2` | Total mass at launch: airframe + motor (loaded). See mass breakdown below. |
| **Prop Wt (lbs)** | `18.3` | Propellant mass consumed during burn |
| **Launch Angle (° from vertical)** | `10.0` | 0 = straight up; positive = tilt off vertical |
| **Launch Alt (ft)** | `4000` | Launch site elevation (MSL); shown on altimeter before launch |
| **Drag Coeff (Cd)** | `0.75` | Pressure drag coefficient — applied to frontal disc area `π(d/2)²` |
| **Skin Friction (Cf)** | `0.004` | Skin friction coefficient — applied to lateral cylinder area `π·d·L`. Painted finish default; see surface finish table in Physics Model. |
| **Body Diameter (in)** | `4.0` | Outer body tube diameter |
| **Body Length (ft)** | `6.2` | Overall airframe length (6 ft 2 in) |

The **derived info** line below the parameters updates live and shows:
`Impulse (Ns) | Burn time (s) | Effective exhaust velocity Ve (m/s) | Specific impulse Isp (s) | Dry mass (kg)`

### Default Configuration — Aerotech N1000W

The defaults model an **Aerotech N1000W** motor in a 12 lb airframe:

| Component | Imperial | Metric |
|---|---|---|
| Airframe (vehicle, dry) | 12.00 lb | 5,443 g |
| Motor — propellant | 18.28 lb | 8,293 g |
| Motor — case (hardware) | 9.87 lb | 4,478 g |
| **Total loaded (launch weight)** | **40.15 lb** | **18,214 g** |
| **Dry mass (burnout weight)** | **21.87 lb** | **9,921 g** |

Motor case mass = motor loaded weight (12,771 g) − propellant (8,293 g) = 4,478 g.

Derived motor parameters for N1000 (N class = 20,480 Ns, 1,000 N avg thrust):

| Parameter | Value |
|---|---|
| Total impulse | 20,480 Ns |
| Average thrust | 1,000 N |
| Burn time | 20.48 s |
| Mass flow rate (ṁ) | 0.405 kg/s |
| Exhaust velocity (Ve) | 2,470 m/s |
| Specific impulse (Isp) | 252 s |

### Parachute Options

| Control | Default | Description |
|---|---|---|
| **Drogue at Apogee** | ✓ checked | Deploy drogue at apogee; 30 ft/s (~9.1 m/s) terminal velocity |
| **Main Chute** | ✓ checked | Deploy main chute at the configured AGL altitude; 5 ft/s (~1.5 m/s) terminal velocity |
| **Deploy Alt (ft)** | `1000` | Height above launch site (AGL) at which the main chute opens |

If neither chute is selected the vehicle descends ballistically, accelerating under gravity and body aerodynamic drag to its natural terminal velocity. All parachute inputs are locked once the server starts.

### TRA Motor Impulse Classes

| Class | Total Impulse (Ns) | Class | Total Impulse (Ns) |
|---|---|---|---|
| A | 2.5 | I | 640 |
| B | 5 | J | 1,280 |
| C | 10 | K | 2,560 |
| D | 20 | L | 5,120 |
| E | 40 | M | 10,240 |
| F | 80 | N | 20,480 |
| G | 160 | O | 40,960 |
| H | 320 | | |

**Example:** `N1000` → Class N = 20,480 Ns total impulse, average thrust = 1,000 N, burn time = 20.5 s

---

## Flight Phases

The emulator steps through six phases automatically. The current phase is shown in the **Live Transmitted Values** panel and on the status bar.

### PRELAUNCH
Server is running but no client has connected yet. The rocket sits static on the launch pad at the configured launch site altitude. Attitude reflects the launch angle (e.g. 10° from vertical → pitch = 80°). No packets are sent until a client connects.

### POWERED *(triggered by first TCP client connection)*
Motor is burning. Physics integrates Newton's second law with time-varying mass:

```
m(t) = m₀ − ṁ·t            (time-varying mass as propellant is consumed)
a    = (F_thrust + Fd) / m(t) − g  (thrust + aerodynamic drag, minus gravity)
```

Thrust is decomposed along the fixed launch angle into vertical and horizontal components. Altitude integrates from the vertical velocity component. Attitude tracks the velocity vector (pitch = angle above horizon).

### COAST
Motor has burned out. Aerodynamic drag and gravity act on the dry mass (airframe + motor case):

```
a = Fd / m_dry − g
```

Vertical speed decreases until it reaches zero at apogee. Attitude continues tracking the velocity vector.

### APOGEE
Vertical speed has reached zero (peak altitude). Physics **continues to integrate** through this phase — gravity and drag still act on the dry mass — so the rocket arcs over naturally rather than freezing at the top. Over approximately **2 seconds** (simulated parachute deployment time), the velocity vector rotates from horizontal through negative pitch as the nose tips earthward:

```
pitch = atan2(Vz, Vh)   [tracks velocity vector continuously through the arc]
```

This produces a smooth parabolic arc with no discontinuity in attitude or velocity at the COAST→APOGEE or APOGEE→DESCENT boundaries.

### DESCENT
The rocket (dry mass = loaded − propellant) descends under gravity. The descent mode depends on the **Parachute** options:

| Mode | Condition | Terminal velocity |
|---|---|---|
| **Drogue** | Drogue checked, above main-chute deploy altitude | 30 ft/s (~9.1 m/s) |
| **Main chute** | Main checked, at or below deploy altitude | 5 ft/s (~1.5 m/s) |
| **Ballistic** | Neither chute checked | Body drag only (Cd + diameter + length) |

**Parachute drag model** (drogue and main chute modes):

```
F_net   = −m_dry·g  +  k_chute·Vz²
a       = F_net / m_dry  =  g·(Vz²/V_term² − 1)
k_chute = m_dry·g / V_term²   (sized so F_drag = m_dry·g at terminal velocity)
```

The rocket decelerates from whatever speed it carries out of APOGEE, converging on the active chute's terminal velocity. If both chutes are enabled, the drogue holds ~30 ft/s until the main deploy altitude is reached, then the main chute slows the rocket to ~5 ft/s.

**Attitude during descent:**

When a chute is active the attitude simulates the rocket hanging nose-down under the canopy:

| Channel | Behaviour |
|---|---|
| Pitch | −75° baseline (nose down) + ±20° pendulum swing at ~0.8 rad/s |
| Roll | ±12° gentle oscillation at ~0.55 rad/s |
| Yaw | Slow 18°/s rotation (parachute spin) |

In ballistic mode the attitude tracks the velocity vector (same as COAST/APOGEE).

### LANDED
When altitude returns to the launch site elevation, the rocket has touched down. The attitude indicator shows a **decaying tumble** as the rocket tips over and comes to rest:

```
pitch = 30° · e^(−0.35t) · sin(3t)
roll  = 25° · e^(−0.35t) · sin(2.2t)
```

The oscillation settles to level in approximately 10 seconds.

---

## Example Flight Profile (N1000W, 40.2 lb loaded, 18.3 lb prop, 10° angle, 4000 ft launch site, both chutes)

Results are approximate; actual values depend on Cd, body geometry, and tick rate. The large wetted surface area of a slender airframe produces significant drag, which limits peak altitude relative to a minimal-frontal-area estimate.

| Phase | Duration | Peak speed | Altitude |
|---|---|---|---|
| POWERED | 20.5 s | ~300 m/s (~Mach 0.9) | — |
| COAST | ~40 s | — | → ~30,000–40,000 ft |
| APOGEE (arc) | ~2 s | 0 → ~−20 m/s | peak (descending) |
| DESCENT — drogue | until 5,000 ft | 30 ft/s | peak → 5,000 ft |
| DESCENT — main | ~200 s | 5 ft/s | 5,000 → 4,000 ft |
| LANDED | ∞ | 0 | 4,000 ft |

---

## GPS Dead Reckoning — NED/ECEF Integration

> **NED coordinate convention used in $INS packets:**
> Vx (index 17) = north m/s, Vy (index 18) = east m/s, Vz (index 19) = vertical up m/s.
> This is NED-frame (North/East/Down with up-positive vertical). Some IMUs output
> ECEF-frame velocities; a per-device coordinate-frame configuration option will be
> added in a future update.

GPS position is computed by the shared `GPSIntegrator` class (`common/gps_integrator.cpp`), which is used by both the emulator and the AHRS receiver app. In the emulator (fix always RTK Fixed), the integrator is seeded every tick so the result is identical to the simulated GPS truth — a useful regression check. In the AHRS app, the integrator dead-reckons from the last known fix and continues smoothly if GPS loses lock (fixType < 2).

### Launch Site

The default launch site is **Blackrock Desert, Lovelock, NV (LLC319050)**:

| Field | Value |
|---|---|
| Latitude | N 40° 52' 59.7"  (40.88325°) |
| Longitude | W 119° 2' 4.7"  (−119.034639°) |
| Altitude MSL | ~4,000 ft / ~1,219 m |
| Satellites in View | 14 |
| Fix Type | 3D Fix |
| RTK Type | RTK Fixed |
| PDOP | 1.20 |

The launch ECEF coordinates are computed from the above at startup via `GPSIntegrator::seed()`, which calls `llaToECEF()` internally.

### Dynamic Position Integration

GPS position is **not fixed**. During flight the emulator calls `GPSIntegrator::integrate()` every tick, updating latitude, longitude, and ECEF position from the rocket's NED velocity. The altitude in the $INS packet always tracks the physics integrator (the same value displayed by the altimeter), so all three coordinates are consistent with the simulated trajectory.

The integration runs in three steps every tick:

#### Step 1 — Convert NED velocity to ECEF displacement

The rocket's world-frame NED velocity `(Vn, Ve, Vd)` is rotated into ECEF using the standard rotation matrix evaluated at the current geodetic position `(φ, λ)`:

```
[ ΔX ]   [ −sinφ·cosλ   −sinλ   −cosφ·cosλ ] [ Vn ]
[ ΔY ] = [ −sinφ·sinλ    cosλ   −cosφ·sinλ ] [ Ve ] · dt
[ ΔZ ]   [  cosφ          0      −sinφ       ] [ Vd ]
```

NED sign convention used:
- `Vn = Vx_ms` (north, from world-frame velocity decomposition in `run()`)
- `Ve = Vy_ms` (east)
- `Vd = −Vz` (down = negative of the physics up-positive vertical speed)

The ECEF position is updated by adding the displacement:

```
X += ΔX,   Y += ΔY,   Z += ΔZ
```

#### Step 2 — Recover latitude and longitude

The updated ECEF coordinates are converted back to geodetic `(lat, lon, alt)` using the **Bowring closed-form approximation**:

```
p       = √(X² + Y²)
θ       = atan2(Z · a,   p · b)          (parametric latitude seed)

φ       = atan2(Z + e'²·b·sin³θ,   p − e²·a·cos³θ)   (geodetic latitude)
λ       = atan2(Y, X)                                   (longitude)

N       = a / √(1 − e²·sin²φ)            (prime vertical radius of curvature)
h       = p / cosφ − N                   (altitude; sinφ form used near poles)
```

WGS84 constants used:

| Constant | Symbol | Value |
|---|---|---|
| Semi-major axis | `a` | 6,378,137.0 m |
| First eccentricity² | `e²` | 0.00669437999014 |
| Semi-minor axis | `b` | 6,356,752.3142 m |
| Second eccentricity² | `e'²` | `(a²−b²)/b²` |

Accuracy: the Bowring approximation is accurate to millimetre level at altitudes below 10 km; well within the requirements of this emulator.

#### Step 3 — Altitude authority and ECEF re-projection

The altitude returned by step 2 is **discarded**. The physics integrator is the authoritative source of altitude (`m_altFt`, integrated from Newton's second law). Mixing two independent integrators for altitude would cause drift.

Instead, the known physics altitude `m_altMslM = m_altFt × 0.3048` is combined with the lat/lon from step 2 to re-project a consistent ECEF:

```
X, Y, Z  =  llaToECEF(lat, lon, m_altMslM)
```

This ensures that ECEF, lat, lon, and altitude in every `$INS` packet are all mutually consistent — any of the three can be recomputed from the other two using the WGS84 formulas.

### Position at Each Flight Phase

| Phase | Behaviour |
|---|---|
| PRELAUNCH | Fixed at launch site; ECEF seeded via `GPSIntegrator::seed(launchLatDeg, launchLonDeg, launchAltFt × 0.3048)` |
| POWERED | Horizontal drift north (launch heading = 0°) proportional to `Vh · sin(launchAngle)` |
| COAST | Continues drifting with residual horizontal velocity |
| APOGEE | Drift continues at low Vh through the arc |
| DESCENT | Minimal horizontal drift (parachute spin does not add horizontal velocity) |
| LANDED | Position frozen at touchdown coordinates |

For a typical N1000W flight on a 10° rail with Vh peaking near 50 m/s, the total horizontal drift from launch to landing is on the order of **2–4 km north** of the launch site.
