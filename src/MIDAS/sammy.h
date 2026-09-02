#pragma once

#include <cmath>
#include "sensor_data.h"

#include <TCAL9539.h>
#include <esp_now.h>

namespace Sammy {
// Configs
constexpr double GPS_TOL{5e-5};         // Tolerance for when turret decides to update its coords
constexpr float manual_adjust{0.05f};   // Controls speed of manual mode
inline float sweep_adjust{0.005f};      // Control speed of automatic mode's sweep
constexpr std::array<uint8_t, 6> broadcastAddress{0xdc, 0x54, 0x75, 0xca, 0xa0, 0x10};  //DC:54:75:CA:A0:10

// WGS84 constants for GPS to ECEF
constexpr double a{6378137.0};
constexpr double f{1.0 / 298.257223563};
constexpr double e2{2*f - f*f};

// struct to hold the turret's desired angles
struct Angles {
    float azimuth;
    float elevation;
};

// struct to hold GPS values as doubles
struct GPS {
    double lat{}, lon{}, alt{};

    // convert to degrees
    static GPS fromMIDAS(const ::GPS& gps) {
        return {gps.latitude * 1.0e-7, gps.longitude * 1.0e-7, gps.altitude};
    }

    // check GPS is not a zero packet
    bool isValid() const {
        return lat != 0.0 || lon != 0.0;
    }
};

/*
 * SamState handles serial inputs and the tracking logic of the turret.
 * The desired angles are accessible through currAngles().
 */
class SamState {
public:
    Angles* currAngles();
    void track(const GPS& sam_gps, const GPS& rocket_gps, const KalmanData& rocket_kalman);
private:
    void handleSerial();
    void updateTurretCoords(const GPS& sam_gps);
    void updateAngles(const GPS& rocket_gps, double rocket_alt);
    Angles angles{};
    bool automatic{false};
    bool lock{false};

    // struct for turret coords
    struct {
        // GPS is stored as radians
        double lat, lon, alt;

        // cache for ENU conversion
        double ecef_x, ecef_y, ecef_z;
        double sin_lat, cos_lat;
        double sin_lon, cos_lon;
    } turret{};
};

};