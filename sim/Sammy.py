import numpy as np
from time import sleep
from util.coords import GPS_to_ECEF, ECEF_to_ENU, norm_pi

class Controller():
    def __init__(self):
        pass

    def compute(self):
        pass

class Sammy():
    def __init__(self, pos_turr:list, pos_rocket:list, motor1, motor2, controller1, controller2):
        # convert rocket to turret basis
        self.pos_turr_GPS = pos_turr
        self.pos_turr_ECEF = GPS_to_ECEF(self.pos_turr_GPS)
        self.pos_rocket_ECEF = GPS_to_ECEF(pos_rocket)
        self.pos_rocket_ENU = ECEF_to_ENU(self.pos_turr_GPS, self.pos_turr_ECEF, self.pos_rocket_ECEF)

        # assign motors
        self.motors = [motor1, motor2] # pitch is motor1, yaw is motor2
        self.controllers = [controller1, controller2] # pitch is controller1, yaw is controller2

        # angle calcs
        pitch = np.arctan2(self.pos_rocket_ENU[2], np.sqrt(self.pos_rocket_ENU[0]**2 + self.pos_rocket_ENU[1]**2))
        yaw = np.arctan2(self.pos_rocket_ENU[1], self.pos_rocket_ENU[0])
        self.ang_init = norm_pi(np.array([pitch, yaw]))
        self.ang_turr = np.zeros(2) # pitch is ang_turr[0], yaw is ang_turr[1]

        self.prev_time = 0

    def update(self, pos_rocket:list, time):
        # convert rocket to turret basis
        self.pos_rocket_ECEF = GPS_to_ECEF(pos_rocket)
        self.pos_rocket_ENU = ECEF_to_ENU(self.pos_turr_GPS, self.pos_turr_ECEF, self.pos_rocket_ECEF)

        # angle calcs
        pitch = np.arctan2(self.pos_rocket_ENU[2], np.sqrt(self.pos_rocket_ENU[0]**2 + self.pos_rocket_ENU[1]**2))
        yaw = np.arctan2(self.pos_rocket_ENU[1], self.pos_rocket_ENU[0])
        
        ang_abs = np.array([pitch, yaw])
        self.ang_k = np.array([
                norm_pi(ang_abs[0] - self.ang_init[0]),
                norm_pi(ang_abs[1] - self.ang_init[1])
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

        # stop oscillations
        if (np.abs(self.error[0]) < self.motors[0].step_size and np.abs(self.error[1]) < self.motors[1].step_size):
            ang_turr_deg = np.degrees(self.ang_turr)
            ang_k_deg = np.degrees(self.ang_k)
            control_output_deg = np.degrees(self.control_output)
            return ang_turr_deg, ang_k_deg, control_output_deg
        
        self.ang_turr[0] += self.control_output[0] * self.motors[0].step_size

        # make sure pitch isn't below 0 degrees
        if(self.ang_turr[0] < 0):
            steps = int(np.ceil(-self.ang_turr[0] / self.motors[0].step_size))
            self.control_output[0] += steps
            self.ang_turr[0] += steps * self.motors[0].step_size
            if (np.abs(self.error[1]) < self.motors[1].step_size):
                ang_turr_deg = np.degrees(self.ang_turr)
                ang_k_deg = np.degrees(self.ang_k)
                control_output_deg = np.degrees(self.control_output)
                return ang_turr_deg, ang_k_deg, control_output_deg
        
        self.ang_turr[1] += self.control_output[1] * self.motors[1].step_size
        
        ang_turr_deg = np.degrees(self.ang_turr)
        ang_k_deg = np.degrees(self.ang_k)
        control_output_deg = np.degrees(self.control_output)

        return ang_turr_deg, ang_k_deg, control_output_deg
    
