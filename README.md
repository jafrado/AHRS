<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/06a6c42e-ad31-4f5f-ad7c-ac634dc5472a" />

# AHRS
Attitude and Heading Reference System provides an interface for compass bearing/heading, and pose (yaw, pitch, roll), as well as navigation basics - GPS position, altitude, speed. Its the guts of what you would use for an aerospace control system or Heads-Up Display (HUD). A reference Inertial Navigation System (INS) firmware is provided for testing and integration using Arduino and the Adafruit/Sparkfun libraries running on a Cortex M0 with the Sparkfun RTK GPS receivers, the Adafruit 9DOF sensor boards, and the Madgwick/Mahoney or NXP sensor fusion libraries are used for 9DOF IMU output in NED format. A Serial and TCP/IP socket based emulator which simulates multiple vehicles (currently a Rocket) is provided for debugging, simulation and verification.

This software is a starting point for integrating both attitude/pose and centimeter level positioning into your system as well as building higher level control systems. 


# Supported Hardware

A suitable AHRS Platform may be build from the below components:
- [Adafruit Feather M0 Basic Proto ](https://www.adafruit.com/product/2772)
- [NXP Precision 9DoF breakout (FXOS8700 3-Axis accel/mag, FXAS21002 3-axis gyro)](https://www.adafruit.com/product/3463)
- [SparkFun GPS-RTK2 Board - ZED-F9P ](https://www.sparkfun.com/products/15136)
- [SparkFun GPS-RTK Board - NEO-M8P-2 ](https://www.sparkfun.com/products/15005)


Follow [Kevin's excellent guide](https://learn.adafruit.com/nxp-precision-9dof-breakout) and [LadyAda's excellent guide on Sensorfusion](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions) to setup the Hardware. You'll be using the I2C port on the M0 for the FXOS7800/FXAS21002 communication interface. Follow [Nathan's excellent guide on RTK GPS Setup](https://learn.sparkfun.com/tutorials/gps-rtk-hookup-guide). For standalone (non NTRIP/networked RTCM corrections), note that the ZED-F9P has two serial ports and should be used as the GPS Master by the ARM Cortex M0. In this configuration, the RTCM correction is sourced from the single output port of the NEO-M8P via the alternate UART port on the the ZED-F9P while the primary ZED-F9P UART is connected to the Arduino M0. This configuration allows for RTK Float mode in a standalone setup.

The following Arduino libraries are included in the firmware, and some modifications have been made to the Sparkfun GPS library for ECEF and NED coordinate systems.


- [Adafruit AHRS](https://github.com/adafruit/Adafruit_AHRS)
- [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor) (modified for use with hardcoded config)
- [Adafruit Sensor Calibration](https://github.com/adafruit/Adafruit_Sensor_Calibration) (modified for use with hardcoded config)
- [Adafruit FXOS8700 Driver](https://github.com/adafruit/Adafruit_FXOS8700)
- [Adafruit FXAS2102C Driver](https://github.com/adafruit/Adafruit_FXAS21002C)
- [Sparkfun uBlox Driver](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library) (modified for ECEF/NED)

All of the libraries and their source code are included in one directory for easy compile/grok of the sources
and it's behavior.

## Hardware Reference

The below references are for the open source hardware designs used in this Arduino sketch ("firmware"):

- [NXP FXOS8700/FXAS2102C](https://github.com/adafruit/Adafruit-FXOS8700-FXAS21002-9-DoF-Breakout-PCB)
- [ATSAMD21G18 ARM Cortex M0](https://github.com/adafruit/Adafruit-Feather-M0-Basic-Proto-PCB)
- [uBlox NEO-M8P high precision, RTK ready GNSS module](https://github.com/sparkfun/Qwiic_GPS-RTK)
- [uBlox ZED-F9P RTK module from ublox](https://github.com/sparkfun/Qwiic_GPS-RTK2)


# Software Prerequisites


## Arduino Firmware

Use Arduino 1.8/8 or later, open the sketch "ahrs.ino" and build. Once programmed, connect the Feather M0 USB port to the hosts system USB port. 
``
## Qt UI
- Qt 5.12.5 or later 
- Windows - Microsoft Visual Studio 2019 Community Edition
- Linux - GNU Compiler Collection (g++) 5.4 and later

To build, simply launch QTCreator and open the project file:

``
qtcreator AHRS.pro
``

Once built, connect to the USB Serial port detected by the system.

## Emulator

A standalone **AHRS INS Emulator** is provided in the `emulator/` directory for testing the Qt UI without physical hardware. It runs a TCP server that streams simulated `$INS` telemetry, and implements a full rocket flight simulation using the [Tsiolkovsky](https://en.wikipedia.org/wiki/Konstantin_Tsiolkovsky) ideal rocket equation.

Defaults are pre-configured for an **Aerotech N1000W** motor in a 12 lb airframe launched from **Blackrock Desert, Lovelock, NV (LLC319050)** at 4,000 ft MSL.

**Features:**
- Configure a TRA motor designation (e.g. `N1000`), loaded weight, propellant weight, launch angle, launch site altitude, drag coefficient, body diameter, and body length
- Derives burn time, mass flow rate, effective exhaust velocity (Ve), specific impulse (Isp), and dry mass automatically from motor designation and weight inputs
- Aerodynamic drag uses total wetted surface area (`π(d/2)² + π·d·L`) for realistic skin-friction and pressure drag scaling with body geometry
- Exponential atmospheric density model (`ρ = 1.22 × 0.9^(alt_m/1000)`) applied at every tick
- Six-phase flight state machine: **PRELAUNCH → POWERED → COAST → APOGEE → DESCENT → LANDED**
- Smooth parabolic arc through apogee — physics integrates continuously, no velocity discontinuity at the top
- Parachute options: drogue at apogee (30 ft/s), main chute at configurable AGL altitude (5 ft/s), or fully ballistic descent
- Descent phase attitude simulates nose-down pendulum swing under canopy, or velocity-vector tracking in ballistic mode
- Landing phase shows a decaying tumble as the rocket tips over and settles
- Static GPS fixed at Blackrock Desert, NV with RTK Fixed lock; ECEF velocity carries motion data

Connect the AHRS app via **Menu → Connect → TCP…** to `localhost` on the emulator's configured port. The simulation launches automatically when the first client connects.

See [`emulator/README.md`](emulator/README.md) for full setup and parameter documentation.

---

# MASI — Mach-enabled Airspeed Indicator

The **MASI** (`MASI.h` / `MASI.cpp`) is a custom `QOpenGLWidget` instrument displayed to the left of the attitude indicator in the instrument panel. It shows total vehicle speed derived from the `$INS` velocity vector and the corresponding Mach number.

## Dial Layout

- **270° sweep** — 7:30 o'clock (zero) clockwise to 4:30 o'clock (maximum)
- **Scale:** 0 – 1,000 in the active unit (see Units below)
- **Major ticks** every 100 units, labelled; medium ticks every 50; minor ticks every 10
- **White needle** pointing to current speed; pivot cap at centre
- **Amber Mach readout box** — dark panel above the pivot, `MACH` label in grey, value in amber/orange matching the Honeywell SI-800 rolling-drum window colour
- **Units label** — displayed at 6-o'clock inside the scale ring at 2× the scale-label font size
- **3D metal bezel** — conical + radial gradient finish matching the attitude indicator and compass

## Speed Input

Speed is derived each telemetry packet from the NED velocity components in the `$INS` stream (tokens 17–19):

```
speed = √(Vx² + Vy² + Vz²)   [m/s, from $INS wire format]
```

The result is converted to display units before being passed to the dial (see Units below). Mach number is always computed in m/s against an altitude-corrected speed of sound:

```
SoS = max(295.0, 340.29 − 0.004 × alt_m)   [m/s, ~ISA lapse rate below 11 km]
Mach = speed_ms / SoS
```

## Units

The MASI tracks the global **Options → Units** setting automatically.

| Setting | Dial reads | Label | Scale max |
|---|---|---|---|
| **Imperial** | Ft/Sec | `Ft/Sec` | 3,000 ft/s ≈ 914 m/s ≈ Mach 2.7 |
| **Metric** | m/s | `m/s` | 1,000 m/s ≈ Mach 2.9 |

The conversion applied before the dial is updated:

```
Imperial:  display_speed = speed_ms × 3.28084   [ft/s]
Metric:    display_speed = speed_ms              [m/s]
```

Mach number is unaffected by the units setting — it is always computed from the raw m/s value.

## Appearance

Font family, label colour, and display scale all follow the AHRS Instruments group in **Options → Appearance**, the same as the attitude indicator and compass. The units label is rendered at **2× the scale-label font size** to remain readable at all window sizes.

---

# Flight Control State Machine

The AHRS Qt UI passively monitors the incoming `$INS` telemetry stream and maintains a flight state machine entirely from observed data — no configuration required. All derived statistics are displayed in the **Flight Statistics panel** on the right side of the status bar, next to the Altitude / Climb Rate panel.

## Flight Statistics Panel

| Field | Description |
|---|---|
| **MAX ALTITUDE** | Highest altitude observed since last connection, in ft (Imperial) or m (Metric) |
| **MAX SPEED** | Highest total speed `√(Vx²+Vy²+Vz²)` observed since launch, in m/s |
| **MAX G** | Highest G-force computed from 3-axis velocity deltas since launch |
| **TIME TO APOGEE** | Elapsed flight time (seconds) at which the maximum altitude was recorded |

All four values reset to `--` automatically when a new connection is established.

## Launch Detection

Launch is confirmed when the computed G-force remains at or above **2 G continuously for 500 ms**. This two-condition gate prevents false triggers from ground handling, transportation vibration, or brief sensor spikes.

```
Each telemetry packet:
  1. Compute instantaneous G from 3-axis velocity delta:
        ax = ΔVx/Δt,  ay = ΔVy/Δt,  az = ΔVz/Δt
        G  = √(ax²+ay²+az²) / 9.80665

  2. G ≥ 2.0:
        — if sustained timer not running → start it
        — if sustained timer ≥ 500 ms   → LAUNCH CONFIRMED

  3. G < 2.0:
        — reset sustained timer (window must be unbroken)
```

G-force is computed on every received packet, including before launch, so the 500 ms window accumulates in real wall-clock time across consecutive telemetry packets.

## State Transitions

```
GROUND (waiting)
    │
    │  G ≥ 2 G sustained for 500 ms
    ▼
FLIGHT (launched)
    │  altitude > previous max  →  update MAX ALTITUDE, TIME TO APOGEE
    │  speed    > previous max  →  update MAX SPEED
    │  G        > previous max  →  update MAX G
    │
    │  (continues until connection is closed or Reset)
    ▼
RESET
    │  new connection established  →  all stats cleared to --
    ▼
GROUND (waiting)
```

## G-Force Computation

G-force is derived from the rate of change of the full 3-axis NED velocity vector reported in the `$INS` stream (tokens 17–19: Vx, Vy, Vz in m/s):

```
G = √( (ΔVx/Δt)² + (ΔVy/Δt)² + (ΔVz/Δt)² ) / g₀
```

where `g₀ = 9.80665 m/s²` and `Δt` is the measured wall-clock interval between consecutive packets. This captures acceleration in all three axes simultaneously, including the dominant axial thrust component and any lateral loads.

> **Note:** This is a derived quantity from GPS/INS velocity data, not a direct accelerometer reading. At low speeds or low packet rates, the estimate may be noisy. For a high-power rocket with 10 Hz telemetry and thrust-to-weight ratio well above 2, the 500 ms gate provides reliable launch discrimination.

## Time to Apogee

The flight timer starts at the moment launch is confirmed. Each time a new maximum altitude is recorded, the current elapsed flight time is captured as the provisional *Time to Apogee*. The value shown is therefore always the flight time at which the **current** peak altitude was set — it updates continuously during ascent and freezes once the rocket begins descending.

## Units

MAX ALTITUDE respects the Imperial/Metric setting under **Options → Units**:
- **Imperial:** feet (ft)
- **Metric:** metres (m)

MAX SPEED and MAX G are always displayed in SI units (m/s and G respectively) regardless of the units setting.

