class PIDController:
    """
    Basic PID controller class for single-axis control.
    """
    def __init__(self, Kp: float, Ki: float, Kd: float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0

    def control(self, error: float, motor, dt: float) -> float:
        """
        Compute the PID output scaled by motor step size.

        Parameters:
            error: Angle error (radians)
            motor: Motor object providing step size
            dt: Time step (seconds)

        Returns:
            Control output in number of motor steps
        """
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error

        output = (
            self.Kp * error +
            self.Ki * self.integral +
            self.Kd * derivative
        )
        return output / motor.step_size
