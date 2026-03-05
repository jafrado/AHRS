#include "gps_integrator.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// WGS84 ellipsoid constants
static const double WGS84_A   = 6378137.0;            // semi-major axis (m)
static const double WGS84_E2  = 0.00669437999014;      // first eccentricity squared
static const double WGS84_B   = 6356752.3142;          // semi-minor axis (m)
static const double WGS84_EP2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B)
                               / (WGS84_B * WGS84_B);  // second eccentricity squared

// ── WGS84 helpers ─────────────────────────────────────────────────────────────

void GPSIntegrator::llaToECEF(double latDeg, double lonDeg, double altM,
                                double &X, double &Y, double &Z)
{
    double lat    = latDeg * M_PI / 180.0;
    double lon    = lonDeg * M_PI / 180.0;
    double sinLat = sin(lat), cosLat = cos(lat);
    double sinLon = sin(lon), cosLon = cos(lon);

    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sinLat * sinLat);

    X = (N + altM) * cosLat * cosLon;
    Y = (N + altM) * cosLat * sinLon;
    Z = (N * (1.0 - WGS84_E2) + altM) * sinLat;
}

void GPSIntegrator::ecefToLLA(double X, double Y, double Z,
                                double &latDeg, double &lonDeg, double &altM)
{
    // Bowring closed-form approximation — accurate to mm at low altitudes
    double p      = sqrt(X * X + Y * Y);
    double theta  = atan2(Z * WGS84_A, p * WGS84_B);

    double lat = atan2(Z + WGS84_EP2 * WGS84_B * pow(sin(theta), 3),
                       p - WGS84_E2  * WGS84_A * pow(cos(theta), 3));
    double lon = atan2(Y, X);

    double sinLat = sin(lat);
    double N      = WGS84_A / sqrt(1.0 - WGS84_E2 * sinLat * sinLat);

    double cosLat = cos(lat);
    altM = (fabs(cosLat) > 1e-10) ? (p / cosLat - N)
                                   : (fabs(Z) / fabs(sinLat) - N * (1.0 - WGS84_E2));

    latDeg = lat * 180.0 / M_PI;
    lonDeg = lon * 180.0 / M_PI;
}

// ── Public API ────────────────────────────────────────────────────────────────

void GPSIntegrator::seed(double latDeg, double lonDeg, double altMslM)
{
    m_latDeg  = latDeg;
    m_lonDeg  = lonDeg;
    m_altMslM = altMslM;
    llaToECEF(m_latDeg, m_lonDeg, m_altMslM, m_ecefX, m_ecefY, m_ecefZ);
    m_seeded  = true;
}

void GPSIntegrator::integrate(double vnMs, double veMs, double altMslM, double dt)
{
    if (!m_seeded) return;

    // NED → ECEF rotation matrix at current geodetic position.
    // Vd = 0: altitude is authoritative (passed in directly); only lat/lon are integrated.
    //
    //  [ dX ]   [ −sinLat·cosLon   −sinLon   −cosLat·cosLon ] [ Vn ]
    //  [ dY ] = [ −sinLat·sinLon    cosLon   −cosLat·sinLon ] [ Ve ] · dt
    //  [ dZ ]   [  cosLat           0         −sinLat        ] [ Vd ]
    //
    double latRad = m_latDeg * M_PI / 180.0;
    double lonRad = m_lonDeg * M_PI / 180.0;
    double sinLat = sin(latRad), cosLat = cos(latRad);
    double sinLon = sin(lonRad), cosLon = cos(lonRad);

    const double Vn = vnMs;
    const double Ve = veMs;
    // Vd = 0 (altitude handled separately)

    m_ecefX += (-sinLat * cosLon * Vn  -  sinLon * Ve) * dt;
    m_ecefY += (-sinLat * sinLon * Vn  +  cosLon * Ve) * dt;
    m_ecefZ += ( cosLat           * Vn)                 * dt;

    // Recover lat/lon from updated ECEF (altitude discarded — authoritative from caller)
    double latNew, lonNew, altDiscard;
    ecefToLLA(m_ecefX, m_ecefY, m_ecefZ, latNew, lonNew, altDiscard);
    m_latDeg  = latNew;
    m_lonDeg  = lonNew;
    m_altMslM = altMslM;

    // Re-project ECEF using authoritative altitude so ECEF stays consistent
    llaToECEF(m_latDeg, m_lonDeg, m_altMslM, m_ecefX, m_ecefY, m_ecefZ);
}
