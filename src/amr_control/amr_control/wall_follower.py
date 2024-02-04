import math
from typing import List, Tuple


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float):
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt
        self._i_error = 0
        self._prev_error = 0

    def compute_commands(self, z_us: List[float], z_v: float, z_w: float) -> Tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
                v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 1.14. Complete the function body with your code (i.e., compute v and w).
        v = 0.5
        w = 0.0
        Kp = 3
        Ki = 0.5
        Kd = 0.3

        # Clamp the ultrasonic readings to a maximum distance of 1.0 m.
        z_us = [min(z, 1.0) for z in z_us]

        left = z_us[1]
        right = z_us[6]
        
    
        error = left - right
        # Compute the integral of the error.
        self._i_error += error * self._dt
        # Compute the derivative of the error.
        d_error = (error - self._prev_error) / self._dt
        # Compute the control law.
        w = Kp * error + Ki * self._i_error + Kd * d_error
        # Check if the robot is too close to the wall.
        front = (z_us[3]+z_us[4])/2
        if front < 0.3:
            v = 0.0
            w = 0.5

        self._prev_error = error

        return v, w
