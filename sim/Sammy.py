import numpy as np

"""

"""
class Controller():
    """

    """
    def __init__(self):
        pass

    """

    """
    def compute(self):
        pass

"""

"""
class Sammy():
    """

    """
    def __init__(self, pos_turr:list, pos_rocket:list, motor1, motor2, controller1, controller2):
        # save turret gps and convert rocket to turret basis
        self.pos_turr_GPS = pos_turr

        self.pos_turr_ECEF = Sammy.GPS_to_ECEF(pos_turr)
        self.pos_rocket_ECEF = Sammy.GPS_to_ECEF(pos_rocket)
        self.pos_rocket_rel = self.pos_rocket_ECEF - self.pos_turr_ECEF
        self.pos_rocket_local = Sammy.ECEF_to_local(self.pos_turr_GPS, self.pos_rocket_rel)

        # assign motors
        self.motors = [motor1, motor2] # pitch is motor1, yaw is motor2
        self.controllers = [controller1, controller2] # pitch is controller1, yaw is controller2

        # angle calcs
        pitch = np.arctan2(self.pos_rocket_local[2], np.sqrt(self.pos_rocket_local[0]**2 + self.pos_rocket_local[1]**2))
        yaw = np.arctan2(self.pos_rocket_local[1], self.pos_rocket_local[0])
        if(yaw < 0):
            yaw += 2 * np.pi
        self.ang_init = np.array([pitch, yaw])
        self.ang_turr = np.zeros(2) # pitch is ang_rel[0], yaw is ang_rel[1]

        self.prev_time = 0

    """
    
    """
    def update(self, pos_rocket:list, time):
        # convert rocket to turret basis
        self.pos_rocket_ECEF = Sammy.GPS_to_ECEF(pos_rocket)
        self.pos_rocket_rel = self.pos_rocket_ECEF - self.pos_turr_ECEF
        self.pos_rocket_local = Sammy.ECEF_to_local(self.pos_turr_GPS, self.pos_rocket_rel)

        # angle calcs
        pitch = np.arctan2(self.pos_rocket_local[2], np.sqrt(self.pos_rocket_local[0]**2 + self.pos_rocket_local[1]**2))
        yaw = np.arctan2(self.pos_rocket_local[1], self.pos_rocket_local[0])
        if(yaw < 0):
            yaw += 2 * np.pi
        self.ang_k = np.array([pitch, yaw]) - self.ang_init
        self.error = self.ang_k - self.ang_turr

        # update dt
        dt = time - self.prev_time
        self.prev_time = time

        # control output
        self.control_output = np.array([
            self.controllers[0].compute(self.error[0], self.motors[0], dt),
            self.controllers[1].compute(self.error[1], self.motors[1], dt)
        ])

        # deadzone control
        self.ang_turr[0] += self.control_output[0] * self.motors[0].step_size
        self.ang_turr[1] += self.control_output[1] * self.motors[1].step_size

        # make sure pitch isn't below 0 degrees
        if(self.ang_turr[0] < 0):
            steps = int(np.ceil(0 - self.ang_turr[0]))
            self.control_output[0] += steps
            self.ang_turr[0] += steps * self.motors[1].step_size
        
        # if both motors are in their deadzones, adjust pitch to be outside of its deadzone
        if(self.ang_turr[1] > self.motors[1].dead_zone[0] and self.ang_turr[1] < self.motors[1].dead_zone[1]):
            if(self.ang_turr[0] > self.motors[0].dead_zone[0] and self.ang_turr[0] < self.motors[0].dead_zone[1]):
                steps = int(np.ceil(self.motors[0].dead_zone[1] - self.ang_turr[0]))
                self.control_output[0] += steps
                self.ang_turr[0] += steps * self.motors[1].step_size
        
        return self.ang_k, self.ang_turr, self.control_output

    """
    Converts GPS (latitude (degrees), longitude (degrees), altitude (m)) into ECEF coordinates (+x, +y, +z)
    centered at the center of the Earth, with x at long = 0, y at long = 90 E, z at lat = 90 N
    """
    @staticmethod
    def GPS_to_ECEF(GPS:list):
        # Input GPS (degrees) -> Output ECEF (SI)
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
    Rotates ECEF coordinates to align with the turret, given the turret's GPS coordinates
    +z is straight up relative to the turret, +x and +y exact orientation don't matter,
    as we are calculating angles relative to starting position.
    """
    @staticmethod
    def ECEF_to_local(pos_turr, ecef_vector):
        """ Convert ECEF vector to local ENU (East, North, Up) frame """
        lat, long, altitude = pos_turr

        # Rotation matrix from ECEF to ENU
        to_3D = lambda theta, phi : [np.cos(theta) * np.cos(phi), np.sin(theta) * np.cos(phi), np.sin(phi)]

        # create a rectangular orthogonal basis with +z on the vertical axis of turret. 
        # absolute direction of +x and +y don't matter
        R = np.array([to_3D(long, lat - np.pi / 2),
        to_3D(long + np.pi / 2, lat - np.pi / 2),
        to_3D(long, lat)
        ]).T
        # transpose to make defition easier
        # inverse to get earth -> turret change of basis matrix
        return np.linalg.inv(R) @ ecef_vector  # Transform vector
    
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
        derivative = (error - self.prev_error)/dt
        PID_calc = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return round(PID_calc / motor.step_size) # return an integer number of steps