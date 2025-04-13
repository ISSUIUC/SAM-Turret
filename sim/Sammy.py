import numpy as np

class Controller():
    def __init__(self):
        pass

    def compute(self):
        pass

class Sammy():
    def __init__(self, pos_turr:list, pos_rocket:list, motor1, motor2, controller1, controller2):
        # convert rocket to turret basis
        self.pos_turr_GPS = pos_turr
        self.pos_turr_ECEF = Sammy.GPS_to_ECEF(self.pos_turr_GPS)
        self.pos_rocket_ECEF = Sammy.GPS_to_ECEF(pos_rocket)
        self.pos_rocket_ENU = Sammy.ECEF_to_ENU(self.pos_turr_GPS, self.pos_turr_ECEF, self.pos_rocket_ECEF)

        # assign motors
        self.motors = [motor1, motor2] # pitch is motor1, yaw is motor2
        self.controllers = [controller1, controller2] # pitch is controller1, yaw is controller2

        # angle calcs
        pitch = np.arctan2(self.pos_rocket_ENU[2], np.sqrt(self.pos_rocket_ENU[0]**2 + self.pos_rocket_ENU[1]**2))
        yaw = np.arctan2(self.pos_rocket_ENU[1], self.pos_rocket_ENU[0])
        self.ang_init = Sammy.norm_pi(np.array([pitch, yaw]))
        self.ang_turr = np.zeros(2) # pitch is ang_turr[0], yaw is ang_turr[1]

        self.prev_time = 0

    def update(self, pos_rocket:list, time):
        # convert rocket to turret basis
        self.pos_rocket_ECEF = Sammy.GPS_to_ECEF(pos_rocket)
        self.pos_rocket_ENU = Sammy.ECEF_to_ENU(self.pos_turr_GPS, self.pos_turr_ECEF, self.pos_rocket_ECEF)

        # angle calcs
        pitch = np.arctan2(self.pos_rocket_ENU[2], np.sqrt(self.pos_rocket_ENU[0]**2 + self.pos_rocket_ENU[1]**2))
        yaw = np.arctan2(self.pos_rocket_ENU[1], self.pos_rocket_ENU[0])
        
        ang_abs = np.array([pitch, yaw])
        self.ang_k = np.array([
                Sammy.norm_pi(ang_abs[0] - self.ang_init[0]),
                Sammy.norm_pi(ang_abs[1] - self.ang_init[1])
        ])
        self.error = self.ang_k - self.ang_turr

        # if no change in time, then don't calculate controls
        # probably can get away with it since our step size is 1.8 degrees
        if(time == self.prev_time):
            ang_turr_deg = np.degrees(self.ang_turr)
            ang_k_deg = np.degrees(self.ang_k)
            return ang_turr_deg, ang_k_deg, np.zeros(2)
        
        # update dt
        dt = time - self.prev_time
        self.prev_time = time

        # control output
        self.control_output = np.array([
            self.controllers[0].compute(self.error[0], self.motors[0], dt),
            self.controllers[1].compute(self.error[1], self.motors[1], dt)
        ])

        if (np.abs(self.error[0]) < self.motors[0].step_size and np.abs(self.error[1]) < self.motors[1].step_size):
            ang_turr_deg = np.degrees(self.ang_turr)
            ang_k_deg = np.degrees(self.ang_k)
            control_output_deg = np.degrees(self.control_output)
            return ang_turr_deg, ang_k_deg, control_output_deg
        
        self.ang_turr[0] += self.control_output[0] * self.motors[0].step_size
        self.ang_turr[1] += self.control_output[1] * self.motors[1].step_size

        # make sure pitch isn't below 0 degrees
        if(self.ang_turr[0] < 0):
            steps = int(np.ceil(-self.ang_turr[0] / self.motors[0].step_size))
            self.control_output[0] += steps
            self.ang_turr[0] += steps * self.motors[0].step_size
        
        ang_turr_deg = np.degrees(self.ang_turr)
        ang_k_deg = np.degrees(self.ang_k)
        control_output_deg = np.degrees(self.control_output)

        return ang_turr_deg, ang_k_deg, control_output_deg

    """
    Used to keep normalize angles between -pi and pi
    """
    @staticmethod
    def norm_pi(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    """
    Converts GPS (latitude (degrees), longitude (degrees), altitude (m)) into ECEF coordinates (+x, +y, +z)
    centered at the center of the Earth, with x at long = 0, y at long = 90 E, z at lat = 90 N
    """
    @staticmethod
    def GPS_to_ECEF(GPS:list):
        # Convert to radians
        lat, long, alt = GPS
        lat = np.radians(lat)
        long = np.radians(long)

        a = 6378137.0                               # Semi-major axis of Earth in meters
        b = 6356752.3142                            # Semi_minor axis of Earth in meters
        e2 = (a**2 - b**2) / a**2                   # Eccentricity squared
        N = a/np.sqrt(1 - e2 * np.sin(lat)**2)      # Prime vertical radius of curvature
        
        x = (N + alt) * np.cos(lat) * np.cos(long)
        y = (N + alt) * np.cos(lat) * np.sin(long)
        z = ((1 - e2) * N + alt) * np.sin(lat)

        return np.array([x, y, z])

    """
    Convert ECEF vector to local ENU (East, North, Up) frame relative to the turret
    """
    @staticmethod
    def ECEF_to_ENU(pos_turr_GPS, pos_turr_ECEF, ECEF_vector):
        lat, lon, alt = pos_turr_GPS

        # Convert to radians
        lat = np.radians(lat)
        lon = np.radians(lon)

        # ECEF to ENU rotation matrix
        # Constructing the local ENU coordinate system (East, North, Up)
        e = np.array([-np.sin(lon), np.cos(lon), 0])                                            # East unit vector
        n = np.array([-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)])     # North unit vector
        u = np.array([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)])       # Up unit vector
        
        # Build the transformation matrix R from ECEF to ENU
        R = np.array([e, n, u]).T
        
        # Subtract turret's position from rocket's position in ECEF
        ECEF_relative = ECEF_vector - pos_turr_ECEF
        
        # Apply the transformation to get ENU coordinates
        ENU_vector = R @ ECEF_relative

        return ENU_vector
    
"""
Klist = [proportional coefficient, integral coefficient, derivative coefficient]
Number of motor steps increases proportionally to current displacement from desired position, 
to accumulated displacement over time (integral), and to the rate of change in error (derivative)
"""
class PID(Controller):
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error, motor, dt):
        self.integral += error
        derivative = (error - self.prev_error) / dt
        PID_calc = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return round(PID_calc / motor.step_size) # return an integer number of steps