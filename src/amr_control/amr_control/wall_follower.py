from typing import List, Tuple
import math


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float):
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt
        self.control = type(
            "control",
            (object,),
            {
                "case": 0,
                "Kp": 8,
                "Kd": 6,
                "LIM_FRONT": 0.5,
                "e_ant": 0,
            },
        )()
        self.limits = type(
            "limits",
            (object,),
            {"lim_v": 0.3, "lim_w": 0.4, "w_turn": 0.6, "side": 0.7},
        )()

    def normalise_sensors(self, z_us: List[float]) -> List[float]:
        """Normalize sensor readings."""
        return [min(value, 1.25) for value in z_us]

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
        z_us = self.normalise_sensors(z_us)

        front = (z_us[3] + z_us[4]) / 2.0
        left1 = (z_us[0] + z_us[1] + z_us[2]) / 3.0
        right = (z_us[7] + z_us[6] + z_us[5]) / 3.0
        diagonaleft = z_us[2]
        diagonalright = z_us[6]

        if front < self.limits.side:
            v = 0.0
            if right < self.limits.side and left1 < self.limits.side:
                w = self.limits.w_turn
            elif right < left1:
                w = self.limits.w_turn
            else:
                w = -self.limits.w_turn
        elif left1 < self.limits.side:
            v = self.limits.lim_v
            w = -self.limits.lim_w
        elif right < self.limits.side:
            v = self.limits.lim_v
            w = self.limits.lim_w
        elif diagonaleft < self.limits.side:
            v = self.limits.lim_v
            w = -self.limits.lim_w
        elif diagonalright < self.limits.side:
            v = self.limits.lim_v
            w = self.limits.lim_w
        else:
            v = self.limits.lim_v
            w = 0.0

        return float(v), float(w)
