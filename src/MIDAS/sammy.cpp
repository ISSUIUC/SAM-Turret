#include "sammy.h"

using namespace Sammy;

/*
 * Return desired angles for the turret
 */
Angles* SamState::currAngles() {
    return &angles;
}

/*
 * Reads the serial input, updating operation mode and angles based on input
 */
void SamState::handleSerial() {
    if (!Serial.available()) 
        return;
    int v = Serial.read();

    // Switch between manual and automatic based on user input
    if (v == '0') {
        Serial.println("Manual Mode");
        automatic = false;
    } else if (v == '1') {
        Serial.println("Automatic Mode");
        automatic = true;
    }

    // manual mode angling
    if (!automatic) {
        if (v == 'w') {
            angles.elevation += manual_adjust;
            if (angles.elevation > M_PI / 2)
                angles.elevation = M_PI / 2;
        } else if (v == 's') {
            angles.elevation -= manual_adjust;
            if (angles.elevation < 0)
                angles.elevation = 0;
        } else if (v == 'd') {
            angles.azimuth -= manual_adjust;
            if (angles.azimuth < -M_PI / 2)
                angles.azimuth = -M_PI / 2;
        } else if (v == 'a') {
            angles.azimuth += manual_adjust;
            if (angles.azimuth > M_PI / 2)
                angles.azimuth = M_PI / 2;
        }
    }
}

/*
 * Handles the tracking logic for the turret by updating the Angles struct of SamState.
 * Sweeps to get a lock if in automatic mode without a lock.
 */
void SamState::track(const GPS& sam_gps, const GPS& rocket_gps, const KalmanData& rocket_kalman) {
    handleSerial();

    if (lock) {
        // calculate angles using the turret's and rocket's coords
        if (sam_gps.isValid() && rocket_gps.isValid()) {
            // update turret coords if they have changed
            if (
                std::abs(sam_gps.lat - turret.lat) > GPS_TOL ||
                std::abs(sam_gps.lon - turret.lon) > GPS_TOL
            ) {
                updateTurretCoords(sam_gps);
            }
            updateAngles(rocket_gps, rocket_kalman.position.px);
        }
    } else {
        // sweep using azimuth until a lock is aquired
        if (sam_gps.isValid() && rocket_gps.isValid()) {
            lock = true;
            updateTurretCoords(sam_gps);
            updateAngles(rocket_gps, rocket_kalman.position.px);
        } else {
            angles.azimuth += sweep_adjust;
            if (angles.azimuth > M_PI / 2) {
                angles.azimuth = M_PI / 2;
                sweep_adjust *= -1.0f;
            } else if (angles.azimuth < -M_PI / 2) {
                angles.azimuth = -M_PI / 2;
                sweep_adjust *= -1.0f;
            }
        }
    }
}

/*
 * Updates the turret's coords and fills its cache for ENU conversions
 */
void SamState::updateTurretCoords(const GPS& sam_gps) {
    // update turret GPS coords and store as radians
    turret.lat = sam_gps.lat * M_PI / 180.0;
    turret.lon = sam_gps.lon * M_PI / 180.0;;
    turret.alt = sam_gps.alt;

    // turret trig cache
    turret.sin_lat = std::sin(turret.lat);
    turret.cos_lat = std::cos(turret.lat);
    turret.sin_lon = std::sin(turret.lon);
    turret.cos_lon = std::cos(turret.lon);

    // conversion to ECEF and update ECEF coords
    double N{a / std::sqrt(1.0 - e2 * turret.sin_lat * turret.sin_lat)};
    turret.ecef_x = (N + turret.alt) * turret.cos_lat * turret.cos_lon;
    turret.ecef_y = (N + turret.alt) * turret.cos_lat * turret.sin_lon;
    turret.ecef_z = (N * (1.0 - e2) + turret.alt) * turret.sin_lat;
}

/*
 * Calculates and updates the desired angles using the inputted rocket's coords and
 * the turret's current coords in SamState
 */
void SamState::updateAngles(const GPS& rocket_gps, double rocket_alt) {
    // rocket trig cache
    double rocket_sin_lat{std::sin(rocket_gps.lat * M_PI / 180.0)};
    double rocket_cos_lat{std::cos(rocket_gps.lat * M_PI / 180.0)};
    double rocket_sin_lon{std::sin(rocket_gps.lon * M_PI / 180.0)};
    double rocket_cos_lon{std::cos(rocket_gps.lon * M_PI / 180.0)};

    // conversion to ECEF
    double N{a / std::sqrt(1.0 - e2 * rocket_sin_lat * rocket_sin_lat)};
    double rocket_ecef_x{(N + rocket_alt) * rocket_cos_lat * rocket_cos_lon};
    double rocket_ecef_y{(N + rocket_alt) * rocket_cos_lat * rocket_sin_lon};
    double rocket_ecef_z{(N * (1.0 - e2) + rocket_alt) * rocket_sin_lat};

    // conversion to ENU
    double dx = rocket_ecef_x - turret.ecef_x;
    double dy = rocket_ecef_y - turret.ecef_y;
    double dz = rocket_ecef_z - turret.ecef_z;

    double east =   - turret.sin_lon * dx 
                    + turret.cos_lon * dy;
    double north =  - turret.sin_lat * turret.cos_lon * dx 
                    - turret.sin_lat * turret.sin_lon * dy 
                    + turret.cos_lat * dz;
    double up =       turret.cos_lat * turret.cos_lon * dx
                    + turret.cos_lat * turret.sin_lon * dy
                    + turret.sin_lat * dz;

    // calculates angles and updates the Angles struct
    angles.azimuth = std::atan2(east, north);
    angles.elevation = std::atan2(up, std::sqrt(east * east + north * north));
    if (angles.elevation < 0.0) angles.elevation = 0.0; // elevation gaurd
}