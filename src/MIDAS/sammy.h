#pragma once

#include <cmath>
#include "sensor_data.h"

namespace Sammy {
    // Tolerance for when turret decides to update its coords
    constexpr double GPS_TOL{5e-5};

    // Controls speed of manual mode and sweep tracking
    float sweep_angle{0.025f};

    // WGS84 constants
    constexpr double a{6378137.0};
    constexpr double f{1.0 / 298.257223563};
    constexpr double e2{2*f - f*f};

    // struct for the turret's angles
    struct Angles {
        float azimuth;
        float elevation;
    } angles{};
    
    // struct for GPS values as doubles
    struct DBLGPS {
        double lat{}, lon{}, alt{};

        static DBLGPS fromMIDAS(const GPS& gps) {
            return { gps.latitude * 1.0e-7, gps.longitude * 1.0e-7, gps.altitude};
        }

        bool isValid() const {
            return lat != 0.0 || lon != 0.0;
        }
    };

    // struct for holding the turret's coords (GPS and ECEF)
    struct Turret {
        double lat, lon, alt;
        double ecef_x, ecef_y, ecef_z;

        // Trig cache
        double sin_lat, cos_lat;
        double sin_lon, cos_lon;
    } turret{};

    // enum for turret tracking states
    enum class TrackingPhase {
        SWEEP,
        LOCK
    } phase{};

    // emum for turret operation mode
    enum class Mode{
        MANUAL,
        AUTOMATIC
    } mode{};

    void SerialHandling(int v);
    void Tracking(DBLGPS& sam_gps, DBLGPS& rocket_gps, KalmanData& rocket_kalman);
    void TurretCalcs(DBLGPS& sam_gps);
    void RocketCalcs(DBLGPS& rocket_gps, float rocket_alt);
};