#pragma once

// emulator_defaults.h
//
// All configurable defaults, GPS origin data, spinbox ranges, widget tooltip
// strings, unit conversions, and UI layout constants for the AHRS INS Emulator.
// Change values here; no other file needs to be touched for tuning.

// ── Application ───────────────────────────────────────────────────────────────

// Main window title string
#define EMUL_WINDOW_TITLE           "AHRS INS Emulator - Rocket Vehicle"

// Minimum window width (pixels) — wide enough to prevent the rocket-parameter
// grid from being squeezed on typical desktop display resolutions
#define EMUL_MIN_WINDOW_WIDTH       640

// ── Network defaults ──────────────────────────────────────────────────────────

// Default TCP port for the outgoing $INS telemetry stream.
// 5555 is an unofficial development/testing port outside the IANA well-known range.
#define EMUL_DEFAULT_TCP_PORT       5555
#define EMUL_TCP_PORT_MIN           1024    // lowest non-reserved port (IANA)
#define EMUL_TCP_PORT_MAX           65535   // maximum 16-bit unsigned port number

// Default simulation tick interval (milliseconds between $INS packets).
// 100 ms = 10 Hz, which matches a typical GNSS receiver update rate.
#define EMUL_DEFAULT_RATE_MS        100
#define EMUL_RATE_MIN_MS            10      // 100 Hz upper limit — typical OS timer resolution
#define EMUL_RATE_MAX_MS            5000    // 0.2 Hz lower limit — still responsive for testing

// ── Launch site — Blackrock Desert, Lovelock, NV (LLC319050) ─────────────────
// Default high-power rocketry launch site used by BALLS and Tripoli Rocketry events.
// Coordinates from Google Maps; elevation from USGS National Elevation Dataset.

// Geodetic latitude, WGS84 degrees North.
// N40° 52' 59.7"  =  40 + 52/60 + 59.7/3600  =  40.88325°
#define EMUL_LAUNCH_LAT_DEG         40.88325

// Geodetic longitude, WGS84 degrees (negative = West).
// W119° 2' 4.7"  =  -(119 + 2/60 + 4.7/3600)  =  -119.034639°
#define EMUL_LAUNCH_LON_DEG         -119.034639

// Launch site elevation, feet MSL.
// Blackrock Desert playa is a flat dry lakebed at approximately 4,000 ft MSL.
#define EMUL_DEFAULT_LAUNCH_ALT_FT  4000
#define EMUL_LAUNCH_ALT_MIN_FT      0
#define EMUL_LAUNCH_ALT_MAX_FT      14000   // ~4,267 m; covers most terrestrial launch sites

// ── Simulated GPS fix — static values transmitted every packet ────────────────
// The emulator broadcasts a fixed, ideal GNSS solution for the whole flight.
// These represent a high-quality open-sky receiver on the Blackrock Desert playa.

// Satellites in view — 14 is typical for a clear-sky open-field site
#define EMUL_GPS_SIV    14

// Fix type: 3 = 3D Fix (position solution from ≥4 satellites)
// $INS protocol values: 0=none, 1=Dead Reckoning, 2=2D, 3=3D, 4=GNSS+DR
#define EMUL_GPS_FIX    3

// RTK correction type: 2 = RTK Fixed (integer ambiguity resolved; cm-level accuracy)
// $INS protocol values: 0=N/A, 1=RTK Float, 2=RTK Fixed
#define EMUL_GPS_RTK    2

// Position dilution of precision — raw wire value (display divides by 10).
// PDOP 12 raw → 1.20 displayed.  PDOP < 2.0 is considered excellent.
#define EMUL_GPS_PDOP   12

// ── Default rocket parameters — Aerotech N1000W in a 12 lb airframe ──────────
// Reference vehicle documented in detail in emulator/README.md.

// Motor designation string in TRA format: letter = impulse class, digits = avg thrust (N).
// N class = 20,480 Ns total impulse; N1000 = 1,000 N average thrust → 20.48 s burn time.
#define EMUL_DEFAULT_MOTOR          "N1000"

// Loaded (launch) weight in pounds:
//   Airframe dry mass:         12.00 lb  (5,443 g)
//   N1000W propellant:         18.28 lb  (8,293 g)
//   N1000W motor case (hardware): 9.87 lb  (4,478 g)
//   Total loaded at liftoff:   40.15 lb  → rounded to 40.2 lb in the UI
#define EMUL_DEFAULT_LOADED_WT_LBS  40.2
#define EMUL_LOADED_WT_MIN_LBS      0.1
#define EMUL_LOADED_WT_MAX_LBS      5000.0

// Propellant weight in pounds:
//   Aerotech N1000W propellant mass = 8,293 g = 18.28 lb → rounded to 18.3 lb
#define EMUL_DEFAULT_PROP_WT_LBS    18.3
#define EMUL_PROP_WT_MIN_LBS        0.1
#define EMUL_PROP_WT_MAX_LBS        4000.0

// Launch angle from vertical (degrees):
//   0° = straight up.  10° is a common competition rail angle; it gives enough
//   horizontal velocity to clear the pad while keeping peak altitude high.
#define EMUL_DEFAULT_LAUNCH_ANGLE_DEG   10.0
#define EMUL_LAUNCH_ANGLE_MIN_DEG       0.0
#define EMUL_LAUNCH_ANGLE_MAX_DEG       89.0    // 90° would be a horizontal launch

// Pressure drag coefficient (Cd), applied to frontal disc area π(d/2)²:
//   0.75 is a reasonable value for a nose-cone + body-tube combination at
//   subsonic to low-supersonic speeds.  See two-term drag model in README.
#define EMUL_DEFAULT_CD     0.75
#define EMUL_CD_MIN         0.10
#define EMUL_CD_MAX         2.00

// Skin friction drag coefficient (Cf), applied to lateral cylinder area π·d·L:
//   Surface finish guide (see also EMUL_CF_TOOLTIP below):
//     Unfinished (raw fibreglass / cardboard)  ≈ 0.008
//     Smooth (sanded)                          ≈ 0.005
//     Painted (default)                        ≈ 0.004
//     Polished                                 ≈ 0.002
#define EMUL_DEFAULT_CF     0.004
#define EMUL_CF_MIN         0.001
#define EMUL_CF_MAX         0.050

// Body tube outer diameter (inches):
//   4 in (101.6 mm) is a standard high-power airframe diameter (LOC, PML, custom tubes)
#define EMUL_DEFAULT_DIAMETER_IN    4.0
#define EMUL_DIAMETER_MIN_IN        0.5
#define EMUL_DIAMETER_MAX_IN        24.0

// Airframe length (feet):
//   6.2 ft = 6 ft 2 in; a typical length for a 4 in diameter N-class single-stage rocket
#define EMUL_DEFAULT_LENGTH_FT      6.2
#define EMUL_LENGTH_MIN_FT          0.5
#define EMUL_LENGTH_MAX_FT          50.0

// ── Default parachute settings ────────────────────────────────────────────────

// Main chute deployment altitude AGL (feet):
//   1,000 ft AGL is a widely used competition requirement.  High enough for safe
//   canopy inflation; low enough to limit horizontal drift in the Blackrock winds.
#define EMUL_DEFAULT_MAIN_CHUTE_ALT_FT  1000
#define EMUL_MAIN_CHUTE_ALT_MIN_FT      0
#define EMUL_MAIN_CHUTE_ALT_MAX_FT      50000

// ── Parachute checkbox label strings ─────────────────────────────────────────
// Terminal velocities shown in the labels match DROGUE_TERM_FPS / MAIN_TERM_FPS
// in vehicle_rocket.cpp (100 ft/s drogue, 5 ft/s main).

#define EMUL_DROGUE_LABEL       "Drogue at Apogee  (100 ft/s)"
#define EMUL_MAIN_CHUTE_LABEL   "Main Chute  (5 ft/s)"

// ── Unit conversions ──────────────────────────────────────────────────────────

// Avoirdupois pounds to kilograms (exact SI definition).
// 1 lb = 0.45359237 kg — International Yard and Pound Agreement (1959).
// Used to convert UI weight inputs (lbs) to SI mass (kg) for the physics model.
#define LBS_TO_KG   0.453592

// Feet to metres (exact).
// 1 ft = 0.3048 m — International Yard and Pound Agreement (1959); NIST Handbook 44.
// Used when converting the vehicle's physics-altitude (feet) to metres for the wire packet.
#define EMUL_FT_TO_M    0.3048

// ── Tooltip strings ───────────────────────────────────────────────────────────

// Skin friction coefficient (Cf) spinbox — shown on mouse hover
#define EMUL_CF_TOOLTIP \
    "Skin friction coefficient applied to lateral body surface area (π·d·L).\n" \
    "Typical values by surface finish:\n" \
    "  Unfinished (raw fibreglass / cardboard)  ≈ 0.008\n" \
    "  Smooth (sanded)                          ≈ 0.005\n" \
    "  Painted (default)                        ≈ 0.004\n" \
    "  Polished                                 ≈ 0.002"

// ── UI layout and style constants ─────────────────────────────────────────────

// Start/Stop button — font size (points) and minimum height (pixels)
#define EMUL_START_BTN_FONT_PT      12
#define EMUL_START_BTN_MIN_HEIGHT   40

// Last-packet display line — maximum characters shown before truncation with "..."
#define EMUL_LAST_LINE_MAX_CHARS    100

// Last-packet monospace font settings
#define EMUL_MONO_FONT_FAMILY       "Courier New"
#define EMUL_MONO_FONT_SIZE_PT      8

// Derived-info label foreground color (muted gray — secondary information)
#define EMUL_DERIVED_LABEL_COLOR    "#777777"

// Start button stylesheet (green = server stopped, ready to start)
#define EMUL_BTN_STYLE_START \
    "QPushButton { background-color: #2e7d32; color: white; border-radius: 4px; }" \
    "QPushButton:hover { background-color: #388e3c; }"

// Stop button stylesheet (red = server running, click to stop)
#define EMUL_BTN_STYLE_STOP \
    "QPushButton { background-color: #c62828; color: white; border-radius: 4px; }" \
    "QPushButton:hover { background-color: #e53935; }"
