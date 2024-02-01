from typing import List, Tuple


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float):
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt

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
        v = 0.8
        w = 0.0
        # Clamp the ultrasonic readings to a maximum distance of 1.0 m.
        z_us = [min(z, 1.0) for z in z_us]
        left = z_us[1]
        right = z_us[6]

        if left < right:
            # turn right
            w = -0.1
        elif right < left:
            # turn left
            w = 0.1
        else:
            w = 0.0

        return v, w
