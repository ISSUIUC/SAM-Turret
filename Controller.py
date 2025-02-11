import numpy as np
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        # Initializes PID controller
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0


    def compute(self, error, dt):
        self.integral += error
        derivative = (error - self.prev_error)/dt
        PID_calc = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return PID_calc



class Controller:
    def __init__(self, turret_lat, turret_long, turret_alt):
        # Input turret's GPS position (degrees)
        self.turret_x, self.turret_y, self.turret_z = Controller.GPS_to_ECEF(turret_lat, turret_long, turret_alt)

        # Initialize PID controller for yaw
        Kp_yaw = 1; Ki_yaw = 1; Kd_yaw = 1
        self.yaw_pid = PID(Kp_yaw, Ki_yaw, Kd_yaw)

        # Initialize PID controller for pitch
        Kp_pitch = 1; Ki_pitch = 1; Kd_pitch = 1
        self.pitch_pid = PID(Kp_pitch, Ki_pitch, Kd_pitch)


    def controller(self, rocket_lat, rocket_long, rocket_alt, current_yaw, current_pitch, dt):
        # Input rocket's GPS position (degrees), current yaw of turret (degrees), curret pitch of turret (degrees)

        # Get relative coordinates by converting to ECEF and subtracting the turret's position
        rocket_x, rocket_y, rocket_z = Controller.GPS_to_ECEF(rocket_lat, rocket_long, rocket_alt)
        relative_x = rocket_x - self.turret_x     # Relative x coord of rocket to turret
        relative_y = rocket_y - self.turret_y     # Relative y coord of rocket to turret
        relative_z = rocket_z - self.turret_z     # Relative z coord of rocket to turret
        
        # Convert to radians
        current_yaw *= math.pi/180
        current_pitch *= math.pi/180

        # Calculate target angles
        target_yaw = math.atan2(relative_y, relative_x)
        target_pitch = math.atan2(relative_z, math.sqrt(relative_x**2 + relative_y**2))

        # Calculate angle errors
        yaw_error = target_yaw - current_yaw
        pitch_error = target_pitch - current_pitch

        # Calculate control values
        yaw_control = self.yaw_pid.compute(yaw_error, dt)
        pitch_control = self.pitch_pid.compute(pitch_error, dt)

        # Convert to motor input
        self.motor_input(yaw_control, pitch_control)


    def motor_input(self, yaw, pitch):
        pass


    @staticmethod
    def GPS_to_ECEF(lat, long, alt): 
        # Input GPS (degrees) -> Output ECEF (SI)

        # Convert to radians
        lat *= math.pi/180
        long *= math.pi/180

        a = 6378137.0                               # Semi-major axis of Earth in meters
        b = 6356752.3142                            # Semi_minor axis of Earth in meters
        f = (a-b)/a                                 # Flatteing factor of an ellipsoid
        e2 = 2*f - f**2                             # Eccentricity squared
        N = a/np.sqrt(1 - e2 * np.sin(lat)**2)      # Prime vertical radius of curvature
        
        x = (N + alt) * np.cos(lat) * np.cos(long)
        y = (N + alt) * np.cos(lat) * np.sin(long)
        z = ((1 - e2) * N + alt) * np.sin(lat)
        return x, y, z
        