import numpy as np

def gps_to_ecef(gps: list[float]) -> np.ndarray:
    """
    Convert GPS coordinates to ECEF (Earth-Centered, Earth-Fixed) coordinates.

    Parameters:
    - gps: [latitude, longitude, altitude] in degrees/meters

    Returns:
    - ECEF position as a NumPy array
    """
    lat, lon, alt = gps
    lat = np.radians(lat)
    lon = np.radians(lon)

    a = 6378137.0  # Earth's semi-major axis in meters
    b = 6356752.3142  # Earth's semi-minor axis in meters
    e2 = (a**2 - b**2) / a**2

    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)

    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = ((1 - e2) * N + alt) * np.sin(lat)

    return np.array([x, y, z])


def ecef_to_local(turret_coords: list[float], ecef_vector: np.ndarray) -> np.ndarray:
    """
    Convert ECEF vector to local ENU (East-North-Up) coordinates.

    Parameters:
    - turret_coords: [lat, lon, alt] in degrees/meters
    - ecef_vector: ECEF-relative position vector

    Returns:
    - ENU-local vector
    """
    lat, lon, _ = turret_coords
    lat = np.radians(lat)
    lon = np.radians(lon)

    to_3D = lambda theta, phi: [np.cos(theta) * np.cos(phi), np.sin(theta) * np.cos(phi), np.sin(phi)]

    R = np.array([
        to_3D(lon, lat - np.pi / 2),         # East
        to_3D(lon + np.pi / 2, lat - np.pi / 2),  # North
        to_3D(lon, lat)                      # Up
    ]).T

    return np.linalg.inv(R) @ ecef_vector
