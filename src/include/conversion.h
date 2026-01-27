#pragma once

#include <cmath>

static const double A = 6378137.0;
static const double B = 6356752.3142;
// #define A 6378137.0                          // Semi-major axis of Earth in meters (Equatorial radius)
// #define F (1.0 / 298.257223563)              // Flattening factor
// #define B (A * (1 - F))                      // Semi-minor axis of Earth in meters (Polar radius)
#define E_SQ ((A * A - B * B) / (A * A))        // Eccentricity squared

void gps_to_ecef(double lat, double lon, double alt, double& ecef_x, double& ecef_y, double& ecef_z);

void ecef_to_enu(double target_x, double target_y, double target_z,
                 double ref_lat, double ref_lon, 
                 double ref_x, double ref_y, double ref_z,
                 double& east, double& north, double& up);

void calculate_pitch_yaw(double east, double north, double up, double& pitch, double& yaw);