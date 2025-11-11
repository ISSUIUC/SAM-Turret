#include "util.h"

// Converts GPS (lat, lon, alt) to ECEF
void gps_to_ecef(double lat, double lon, double alt, double& x, double& y, double& z) {
    lat *= M_PI / 180.0;
    lon *= M_PI / 180.0;
    double N = A / std::sqrt(1 - E_SQ * std::sin(lat) * std::sin(lat));
    x = (N + alt) * std::cos(lat) * std::cos(lon);
    y = (N + alt) * std::cos(lat) * std::sin(lon);
    z = ((1 - E_SQ) * N + alt) * std::sin(lat);
}

// Converts ECEF to ENU
void ecef_to_enu(double x, double y, double z,
                 double ref_lat, double ref_lon, double ref_x, double ref_y, double ref_z,
                 double& east, double& north, double& up) {
    ref_lat *= M_PI / 180.0;
    ref_lon *= M_PI / 180.0;

    double dx = x - ref_x;
    double dy = y - ref_y;
    double dz = z - ref_z;

    east  = -std::sin(ref_lon) * dx + std::cos(ref_lon) * dy;
    north = -std::sin(ref_lat) * std::cos(ref_lon) * dx
            - std::sin(ref_lat) * std::sin(ref_lon) * dy
            + std::cos(ref_lat) * dz;
    up    = std::cos(ref_lat) * std::cos(ref_lon) * dx
            + std::cos(ref_lat) * std::sin(ref_lon) * dy
            + std::sin(ref_lat) * dz;
}