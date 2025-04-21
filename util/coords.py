import numpy as np
"""
Used to normalize angles between -pi and pi
    """
def norm_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
"""
    Converts GPS (latitude (degrees), longitude (degrees), altitude (m)) into ECEF coordinates (+x, +y, +z)
    centered at the center of the Earth, with x at long = 0, y at long = 90 E, z at lat = 90 N
    """

def GPS_to_ECEF(GPS: list):
    # Convert to radians
    lat, long, alt = GPS
    lat = np.radians(lat)
    long = np.radians(long)

    a = 6378137.0                               # Semi-major axis of Earth in meters
    b = 6356752.3142                            # Semi_minor axis of Earth in meters
    e2 = (a**2 - b**2) / a**2                   # Eccentricity squared
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)    # Prime vertical radius of curvature

    x = (N + alt) * np.cos(lat) * np.cos(long)
    y = (N + alt) * np.cos(lat) * np.sin(long)
    z = ((1 - e2) * N + alt) * np.sin(lat)

    return np.array([x, y, z])

"""
    Convert ECEF vector to local ENU (East, North, Up) frame relative to the turret
"""
def ECEF_to_ENU(pos_turr_GPS, pos_turr_ECEF, ECEF_vector):
    lat, lon, _ = pos_turr_GPS

    # Convert to radians
    lat = np.radians(lat)
    lon = np.radians(lon)

    # ECEF to ENU rotation matrix
    # Constructing the local ENU coordinate system (East, North, Up)
    e = np.array([-np.sin(lon), np.cos(lon), 0])                                            # East unit vector
    n = np.array([-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)])     # North unit vector
    u = np.array([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)])       # Up unit vector

    # Build the transformation matrix R from ECEF to ENU
    R = np.array([e, n, u])

            # Subtract turret's position from rocket's position in ECEF
    ECEF_relative = ECEF_vector - pos_turr_ECEF

    # Apply the transformation to get ENU coordinates
    ENU_vector = R @ ECEF_relative

    return ENU_vector
