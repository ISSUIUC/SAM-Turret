import numpy as np

class Motor:
    def __init__(self, step_size, gear_ratio, actuation_time):
        self.step_size = np.radians(step_size)
        self.gear_ratio = gear_ratio
        self.actuation_time = actuation_time