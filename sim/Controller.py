import numpy as np

class Controller():
    def __init__(self, pos_turr:list, pos_rocket:list, motor1, motor2, controller1, controller2):
        pass
    def update(self, pos_rocket:list, time):
        pass

class Turret_Controller(Controller):
    """
    This class is the controller for Sammy :)
    """
    def __init__(self, pos_turr:list, pos_rocket:list, motor1, motor2, controller1, controller2):
        self.pos_turr = pos_turr
        self.pos_ECEF = Controller.GPS_to_ECEF(pos_turr)
        self.pos_rocket_ECEF = Controller.GPS_to_ECEF(pos_rocket)
        self.pos_rocket_rel = self.pos_rocket_ECEF - self.pos_ECEF
        self.pos_rocket_local = self.ECEF_to_local(self.pos_turr, self.pos_rocket_rel)
        self.motors = [motor1, motor2] # pitch is motor1, yaw is motor2
        self.controllers = [controller1, controller2] # pitch is controller1, yaw is controller2
        pitch = np.atan2(self.pos_rocket_local[2], np.sqrt(self.pos_rocket_local[0]**2 + self.pos_rocket_local[1]**2))
        yaw = np.atan2(self.pos_rocket_local[1], self.pos_rocket_local[0])
        if(yaw < 0):
            yaw += 2 * np.pi
        self.ang_init = np.array([pitch, yaw])
        self.ang_turr = np.zeros(2) # pitch is ang_rel[0], yaw is ang_rel[1]
        self.prev_time = 0

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

    def update(self, pos_rocket:list, time):
        # angle calcs
        self.pos_rocket_ECEF = Controller.GPS_to_ECEF(pos_rocket)
        self.pos_rocket_rel = self.pos_rocket_ECEF - self.pos_ECEF
        self.pos_rocket_local = self.ECEF_to_local(self.pos_turr, self.pos_rocket_rel)
        pitch = np.atan2(self.pos_rocket_local[2], np.sqrt(self.pos_rocket_local[0]**2 + self.pos_rocket_local[1]**2))
        yaw = np.atan2(self.pos_rocket_local[1], self.pos_rocket_local[0])
        if(yaw < 0):
            yaw += 2 * np.pi
        self.ang_k = np.array([pitch, yaw]) - self.ang_init
        self.error = self.ang_k - self.ang_turr
        # update dt
        dt = time - self.prev_time
        self.prev_time = time
        # control output
        self.control_output = np.array([
            self.controllers[0].control(self.error[0], self.motors[0], dt),
            self.controllers[1].control(self.error[1], self.motors[1], dt)
        ])
        # deadzone control
        self.ang_turr[0] += self.control_output[0] * self.motors[0].step_size
        self.ang_turr[1] += self.control_output[1] * self.motors[1].step_size
        if(self.ang_turr[0] < 0):
            steps = int(np.ceil(0 - self.ang_turr[0]))
            self.control_output[0] += steps
            self.ang_turr[0] += steps * self.motors[1].step_size
        if(self.ang_turr[1] > self.motors[1].deadzone[0] and self.ang_turr[1] < self.motors[1].deadzone[1]):
            if(self.ang_turr[0] > self.motors[0].deadzone[0] and self.ang_turr[0] < self.motors[0].deadzone[1]):
                steps = int(np.ceil(self.motors[0].deadzone[1] - self.ang_turr[0]))
                self.control_output[0] += steps
                self.ang_turr[0] += steps * self.motors[1].step_size
        else:
            self.ang_turr[0] += self.control_output[0] * self.motors[1].step_size

    """
    @brief: Converts GPS coordinates to ECEF coordinates

    @return: numpy array of ECEF coordinates
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

class PID():
    def __init__(self, K:list):
        self.Kp, self.Ki, self.Kd = K
        self.prev_error = 0
        self.integral = 0

    def control(self, error, motor, dt):
        self.integral += error
        derivative = (error - self.prev_error)/dt
        PID_calc = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return round(PID_calc / motor.step_size)




'''
class Controller:
    def __init__(self, turret_lat, turret_long, turret_alt, rocket_lat, rocket_long, rocket_alt, time):
        # Initialize time
        self.prev_time = time

        # Input turret's GPS position (degrees)
        self.turret_x, self.turret_y, self.turret_z = Controller.GPS_to_ECEF(turret_lat, turret_long, turret_alt)

        # Get relative coordinates by converting to ECEF and subtracting the turret's position
        rocket_x, rocket_y, rocket_z = Controller.GPS_to_ECEF(rocket_lat, rocket_long, rocket_alt)
        relative_x = rocket_x - self.turret_x     # Relative x coord of rocket to turret
        relative_y = rocket_y - self.turret_y     # Relative y coord of rocket to turret
        relative_z = rocket_z - self.turret_z     # Relative z coord of rocket to turret

        # Initialize turret angles
        self.turret_yaw = np.atan2(relative_y, relative_x)
        self.turret_pitch = np.atan2(relative_z, np.sqrt(relative_x**2 + relative_y**2))

        self.relative_yaw = 0
        self.relative_pitch = 0

        # Initialize PID controller for yaw
        Kp_yaw = 1; Ki_yaw = 1; Kd_yaw = 1
        self.yaw_pid = PID(Kp_yaw, Ki_yaw, Kd_yaw, turret_lat, turret_long, turret_alt, rocket_lat, rocket_long, rocket_alt, time)

        # Initialize PID controller for pitch
        Kp_pitch = 1; Ki_pitch = 1; Kd_pitch = 1
        self.pitch_pid = PID(Kp_pitch, Ki_pitch, Kd_pitch, turret_lat, turret_long, turret_alt, rocket_lat, rocket_long, rocket_alt, time)

        # Initialize motor step size (degrees) and time between steps (s)
        self.step_size = 1.8
        self.step_time = 1

        # Convert degrees to radians
        self.step_size *= np.pi/180
        
    def controller(self, rocket_lat, rocket_long, rocket_alt, time):
        # Input rocket's GPS position (degrees), current yaw of turret (degrees), curret pitch of turret (degrees)

        # Get relative coordinates by converting to ECEF and subtracting the turret's position
        rocket_x, rocket_y, rocket_z = Controller.GPS_to_ECEF(rocket_lat, rocket_long, rocket_alt)
        relative_x = rocket_x - self.turret_x     # Relative x coord of rocket to turret
        relative_y = rocket_y - self.turret_y     # Relative y coord of rocket to turret
        relative_z = rocket_z - self.turret_z     # Relative z coord of rocket to turret

        # Calculate target angles
        target_yaw = np.atan2(relative_y, relative_x)
        target_pitch = np.atan2(relative_z, np.sqrt(relative_x**2 + relative_y**2))

        # Calculate angle errors
        yaw_error = target_yaw - self.turret_yaw
        pitch_error = target_pitch - self.turret_pitch

        # Calculate dt
        dt = time - self.prev_time
        self.prev_time = time

        # Calculate control values
        yaw_steps = self.yaw_pid.compute(yaw_error, self.step_size, dt)
        pitch_steps = self.pitch_pid.compute(pitch_error, self.step_size, dt)

        # Update current angles
        self.update_turret_angles(yaw_steps, pitch_steps)

        return (target_yaw - self.turret_yaw, target_pitch - self.turret_pitch)

    def update_turret_angles(self, yaw_steps, pitch_steps):
        self.turret_yaw += yaw_steps * self.step_size
        self.relative_yaw += yaw_steps * self.step_size
        self.turret_pitch += pitch_steps * self.step_size
        self.relative_pitch += pitch_steps * self.step_size
        if(self.relative_yaw > 160*np.pi/180 & self.relative_yaw < 200*np.pi/180 & self.relative_pitch < 15*np.pi/180):
            pass

    @staticmethod
    def GPS_to_ECEF(lat, long, alt): 
        # Input GPS (degrees) -> Output ECEF (SI)

        # Convert to radians
        lat *= np.pi/180
        long *= np.pi/180

        a = 6378137.0                               # Semi-major axis of Earth in meters
        b = 6356752.3142                            # Semi_minor axis of Earth in meters
        f = (a-b)/a                                 # Flatteing factor of an ellipsoid
        e2 = 2*f - f**2                             # Eccentricity squared
        N = a/np.sqrt(1 - e2 * np.sin(lat)**2)      # Prime vertical radius of curvature
        
        x = (N + alt) * np.cos(lat) * np.cos(long)
        y = (N + alt) * np.cos(lat) * np.sin(long)
        z = ((1 - e2) * N + alt) * np.sin(lat)
        return x, y, z
'''