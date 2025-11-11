from sammy import Controller
"""
Klist = [proportional coefficient, integral coefficient, derivative coefficient]
Number of motor steps increases proportionally to current displacement from desired position, 
to accumulated displacement over time (integral), and to the rate of change in error (derivative)
"""
class PID(Controller):
    def __init__(self, K_list : list):
        self.Kp, self.Ki, self.Kd = K_list
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error, motor, dt):
        self.integral += error
        derivative = (error - self.prev_error) / dt
        PID_calc = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return round(PID_calc / (motor.step_size * motor.gear_ratio)) # return an integer number of steps
