import numpy as np
from coordinates import gps_to_ecef, ecef_to_local

class TurretController:
    """
    This class is the main controller for the turret, responsible for tracking a rocket
    in the sky and issuing commands to the pitch and yaw motors to follow it.
    """
    def __init__(self, pos_turret: list, pos_rocket: list, motor_pitch, motor_yaw, controller_pitch, controller_yaw):
        self.pos_turret = pos_turret
        self.ecef_turret = gps_to_ecef(pos_turret)
        self.ecef_rocket = gps_to_ecef(pos_rocket)

        # Initial transformation to ENU
        rel_ecef = self.ecef_rocket - self.ecef_turret
        self.rocket_local = ecef_to_local(pos_turret, rel_ecef)

        self.motors = [motor_pitch, motor_yaw]
        self.controllers = [controller_pitch, controller_yaw]

        pitch = np.arctan2(self.rocket_local[2], np.linalg.norm(self.rocket_local[:2]))
        yaw = np.arctan2(self.rocket_local[1], self.rocket_local[0])
        if yaw < 0:
            yaw += 2 * np.pi

        self.initial_angles = np.array([pitch, yaw])
        self.turret_angles = np.zeros(2)
        self.prev_time = 0

    def update(self, pos_rocket: list, time: float):
        self.ecef_rocket = gps_to_ecef(pos_rocket)
        rel_ecef = self.ecef_rocket - self.ecef_turret
        self.rocket_local = ecef_to_local(self.pos_turret, rel_ecef)

        pitch = np.arctan2(self.rocket_local[2], np.linalg.norm(self.rocket_local[:2]))
        yaw = np.arctan2(self.rocket_local[1], self.rocket_local[0])
        if yaw < 0:
            yaw += 2 * np.pi

        current_angles = np.array([pitch, yaw]) - self.initial_angles
        error = current_angles - self.turret_angles

        dt = time - self.prev_time
        self.prev_time = time

        control_output = np.array([
            self.controllers[0].control(error[0], self.motors[0], dt),
            self.controllers[1].control(error[1], self.motors[1], dt)
        ])

        # Update turret angles and handle deadzones
        self.turret_angles += control_output * np.array([
            self.motors[0].step_size,
            self.motors[1].step_size
        ])

        # Enforce mechanical constraints (e.g., deadzone limits)
        if self.turret_angles[0] < 0:
            steps = int(np.ceil(-self.turret_angles[0]))
            self.turret_angles[0] += steps * self.motors[0].step_size
        if (self.motors[1].deadzone[0] < self.turret_angles[1] < self.motors[1].deadzone[1]):
            if (self.motors[0].deadzone[0] < self.turret_angles[0] < self.motors[0].deadzone[1]):
                steps = int(np.ceil(self.motors[0].deadzone[1] - self.turret_angles[0]))
                self.turret_angles[0] += steps * self.motors[0].step_size
