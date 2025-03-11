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
        pass

    """
    
    """
    def update(self, pos_rocket:list, time):
        pass

    """
    Converts GPS (latitude (degrees), longitude (degrees), altitude (m)) into ECEF coordinates (+x, +y, +z)
    centered at the center of the Earth, with x at long = 0, y at long = 90 E, z at lat = 90 N
    """
    @staticmethod
    def GPS_to_ECEF(GPS:list):
        pass

    """
    Rotates ECEF coordinates to align with the turret, given the turret's GPS coordinates
    +z is straight up relative to the turret, +x and +y exact orientation don't matter,
    as we are calculating angles relative to starting position.
    """
    @staticmethod
    def ECEF_to_local(pos_turr, ecef_vector):
        pass
    
"""
Klist = [proportional coefficient, integral coefficient, derivative coefficient]
Number of motor steps increases proportionally to current displacement from desired position, 
to accumulated displacement over time (integral), and to the rate of change in error (derivative)
"""
class PID(Controller):
    def __init__(self, Klist:list):
        self.Kp, self.Ki, self.Kd = Klist
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error, motor, dt):
        self.integral += error
        derivative = (error - self.prev_error)/dt
        PID_calc = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return round(PID_calc / motor.step_size) # return an integer number of steps