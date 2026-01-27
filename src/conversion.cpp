#include "include/conversion.h"

// Converts GPS (lat, lon, alt) to ECEF
void gps_to_ecef(double lat, double lon, double alt, double& ecef_x, double& ecef_y, double& ecef_z) {
    lat *= M_PI / 180.0;
    lon *= M_PI / 180.0;

    double N = A / std::sqrt(1 - E_SQ * std::sin(lat) * std::sin(lat));

    ecef_x = (N + alt) * std::cos(lat) * std::cos(lon);
    ecef_y = (N + alt) * std::cos(lat) * std::sin(lon);
    ecef_z = ((1 - E_SQ) * N + alt) * std::sin(lat);
}

// Converts ECEF to ENU
void ecef_to_enu(double target_x, double target_y, double target_z,
                 double ref_lat, double ref_lon, 
                 double ref_x, double ref_y, double ref_z,
                 double& east, double& north, double& up) {
    ref_lat *= M_PI / 180.0;
    ref_lon *= M_PI / 180.0;

    double dx = target_x - ref_x;
    double dy = target_y - ref_y;
    double dz = target_z - ref_z;

    east  = - std::sin(ref_lon) * dx 
            + std::cos(ref_lon) * dy;
    north = - std::sin(ref_lat) * std::cos(ref_lon) * dx
            - std::sin(ref_lat) * std::sin(ref_lon) * dy
            + std::cos(ref_lat) * dz;
    up    =   std::cos(ref_lat) * std::cos(ref_lon) * dx
            + std::cos(ref_lat) * std::sin(ref_lon) * dy
            + std::sin(ref_lat) * dz;
}

// Calculate pitch and yaw
void calculate_pitch_yaw(double east, double north, double up, double& pitch, double& yaw) {
    pitch = std::atan2(up, std::sqrt(east * east + north * north));
    yaw = std::atan2(north, east);
}