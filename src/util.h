#pragma once

#include <cmath>

#define A 6378137.0                         // Equatorial radius
#define F (1.0 / 298.257223563)             // Flattening factor
#define B (A * (1 - F))                     // Polar radius
#define E_SQ ((A * A - B * B) / (A * A))    // Eccentricity squared

void gps_to_ecef(double lat, double lon, double alt, double& x, double& y, double& z);

void ecef_to_enu(double x, double y, double z,
                 double ref_lat, double ref_lon, double ref_x, double ref_y, double ref_z,
                 double& east, double& north, double& up);