#include "sammy.h"

/*
 * Reads serial and updates mode and angles accordingly
 */
void Sammy::SerialHandling(int v) {
    // Switch between manual and automatic based on user input
    if (v == '0') {
        Serial.println("Manual Mode");
        mode = Mode::MANUAL;
    } else if (v == '1') {
        Serial.println("Automatic Mode");
        mode = Mode::AUTOMATIC;
    }
    // manual mode angling
    if (mode == Mode::MANUAL) {
        if (v == 'w') {
            angles.elevation += sweep_angle;
            if (angles.elevation > M_PI / 2)
                angles.elevation = M_PI / 2;
        } else if (v == 's') {
            angles.elevation -= sweep_angle;
            if (angles.elevation < 0)
                angles.elevation = 0;
        } else if (v == 'd') {
            angles.azimuth -= sweep_angle;
            if (angles.azimuth < -M_PI / 2)
                angles.azimuth = -M_PI / 2;
        } else if (v == 'a') {
            angles.azimuth += sweep_angle;
            if (angles.azimuth > M_PI / 2)
                angles.azimuth = M_PI / 2;
        }
    }
}
/*
 * Tracking logic for the turret
 */
void Sammy::Tracking(DBLGPS& sam_gps, DBLGPS& rocket_gps, KalmanData& rocket_kalman) {
    switch (phase) {
    // sweeps using only azimuth to get a lock on the rocket
    case TrackingPhase::SWEEP:
        if (sam_gps.isValid() && rocket_gps.isValid()) {
            phase = TrackingPhase::LOCK;
            TurretCalcs(sam_gps);
            RocketCalcs(rocket_gps, rocket_kalman.position.px);
        } else {
            angles.azimuth += sweep_angle;
            if (angles.azimuth > M_PI / 2) {
                angles.azimuth = M_PI / 2;
                sweep_angle *= -1.0f;
            } else if (angles.azimuth < -M_PI / 2) {
                angles.azimuth = -M_PI / 2;
                sweep_angle *= -1.0f;
            }
        }
        break;
    // calculate angle using turret and rocket coords
    case TrackingPhase::LOCK:
        if (sam_gps.isValid() && rocket_gps.isValid()) {
            // update turret coords if they have changed
            if (
                std::abs(sam_gps.lat - turret.lat) > GPS_TOL ||
                std::abs(sam_gps.lon - turret.lon) > GPS_TOL
            ) {
                TurretCalcs(sam_gps);
            }
            RocketCalcs(rocket_gps, rocket_kalman.position.px);
        }
        break;
    }
}

/*
 * Updates the turret's coords and fills trig cache for ENU conversion
 */
void Sammy::TurretCalcs(DBLGPS& sam_gps) {
    // update turret GPS coords
    turret.lat = sam_gps.lat;
    turret.lon = sam_gps.lon;
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
 * Takes in the rocket's coords, converts GPS -> ECEF -> ENU based on the turret,
 * and updates the Angles struct with the new angles for azimuth and elevation
 */
void Sammy::RocketCalcs(DBLGPS& rocket_gps, double rocket_alt) {
    // rocket trig cache
    double rocket_sin_lat{std::sin(rocket_gps.lat)};
    double rocket_cos_lat{std::cos(rocket_gps.lat)};
    double rocket_sin_lon{std::sin(rocket_gps.lon)};
    double rocket_cos_lon{std::cos(rocket_gps.lon)};

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