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
        self._cant_giro = 0

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
        v = 0.0
        w = 0.0
        Kp = 3
        Ki = 2
        Kd = 1.8

        # Clamp the ultrasonic readings to a maximum distance of 1.0 m.
        z_us = [min(z, 1.5) for z in z_us]

        right = min(z_us[7], z_us[8])
        left = min(z_us[0], z_us[15])

        if left < 1.4 and right < 1.4 and self._cant_giro == 0:
            error = min(z_us[0], z_us[1]) - min(z_us[6], z_us[7])
            # Compute the integral of the error.
            self._i_error += error * self._dt
            # Compute the derivative of the error.
            d_error = (error - self._prev_error) / self._dt
            # Compute the control law.
            w = Kp * error + Ki * self._i_error + Kd * d_error
            self._prev_error = error
            v = 0.5

        elif left == 1.5 and right < 1.4 and self._cant_giro == 0:
            self._i_error = 0.0
            # Wall at right
            error = z_us[8] - z_us[7]
            w = 1 * error
            v = 0.5

        elif left < 1.4 and right == 1.5 and self._cant_giro == 0:
            self._i_error = 0.0
           # Wall at left
            error = z_us[0] - z_us[15]
            w = 1 * error
            v = 0.5            

        elif self._cant_giro == 0:
            self._i_error = 0.0
            # We have only a wall before us not at left or right
            w = 1.1
            v = 0.5

        # Check if the robot is too close to the wall.
        front = (z_us[2] + z_us[5]) / 2
        if front < 0.4:
            v = 0.05
            # Turning taking into account the walls
            if left == 1.5 and right < 1.4:
                # Right wall
                 w = 0.75
            elif left < 1.4 and right == 1.5:
                # Left wall
                w = -0.75
            else:
                w = 0.75

            self._cant_giro += abs(z_w*self._dt)
            if self._cant_giro > (math.pi / 2 - 0.1):
                self._cant_giro = 0

        
        if self._cant_giro != 0 and front >= 0.4:
            v = 0.05
            # Turning taking into account the walls
            if left == 1.5 and right < 1.4:
                # Right wall
                 w = 0.75
            elif left < 1.4 and right == 1.5:
                # Left wall
                w = -0.75
            else:
                w = 0.75

            self._cant_giro += abs(z_w*self._dt)
            if self._cant_giro > (math.pi / 2 - 0.1):
                self._cant_giro = 0

        return v, w