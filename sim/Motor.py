import numpy as np

class Motor:
    def __init__(self, step_size, actuation_time):
        self.step_size = np.radians(step_size)
        self.actuation_time = actuation_time