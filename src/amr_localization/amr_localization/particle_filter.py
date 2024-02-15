import datetime
import math
import numpy as np
import os
import pytz

from amr_localization.map import Map
from sklearn.cluster import DBSCAN
from matplotlib import pyplot as plt
from typing import List, Tuple


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        sensors: List[Tuple[float, float, float]],
        sensor_range: float,
        particle_count: int,
        sigma_v: float = 0.15,
        sigma_w: float = 0.75,
        sigma_z: float = 0.25,
    ):
        """Particle filter class initializer.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            sensors: Robot sensors' pose in the robot coordinate frame (x, y, theta) [m, m, rad].
            sensor_range: Sensor measurement range [m].
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].

        """
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._particle_count: int = particle_count
        self._sensors: List[Tuple[float, float, float]] = sensors
        self._sensor_range: float = sensor_range
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._iteration: int = 0

        self._map = Map(map_path, sensor_range, compiled_intersect=True, use_regions=True)
        self._particles = self._init_particles(particle_count)
        self._ds, self._phi = self._init_sensor_polar_coordinates(sensors)
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def compute_pose(self) -> Tuple[bool, Tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        localized: bool = False
        pose: Tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))
        # pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        # TODO: 2.10. Complete the missing function body with your code.
        # Compute the clusters
    
        db = DBSCAN(eps=0.3, min_samples=10).fit(self._particles)
        labels = db.labels_
        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)
        print(f"Number of clusters: {n_clusters_}, noise: {n_noise_}")
        # # If there is only one cluster
        if n_clusters_ == 1:
            localized = True
            print("---------------------Localized---------------------")
            # Compute the pose estimate as the mean of the particles
            points_of_cluster_0 = self._particles[labels == 0]
            centroid_of_cluster_0 = np.mean(points_of_cluster_0, axis=0) 
            print(centroid_of_cluster_0)
            pose = (centroid_of_cluster_0[0], centroid_of_cluster_0[1], centroid_of_cluster_0[2])
            print("Localization pose:", pose)
            # Keep 100 particles for pose tracking
            self._particle_count = 100
        return localized, pose

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].

        """
        self._iteration += 1

        # TODO: 2.5. Complete the function body with your code (i.e., replace the pass statement).
        for i, particle in enumerate(self._particles):
            # Compute new particle pose
            x = particle[0] + (v + np.random.normal(0, self._sigma_v)) * self._dt * math.cos(
                particle[2]
            )
            y = particle[1] + (v + np.random.normal(0, self._sigma_w)) * self._dt * math.sin(
                particle[2]
            )
            theta = particle[2] + (w + np.random.normal(0, self._sigma_w)) * self._dt
            intersection, distance = self._map.check_collision(((particle[0], particle[1]), (x, y)))

            if not intersection:
                self._particles[i] = (x, y, theta)
            else:
                self._particles[i] = (intersection[0], intersection[1], theta)
        pass

    def resample(self, measurements: List[float]) -> None:
        """Samples a new set of particles.

        Args:
            measurements: Sensor measurements [m].

        """
        # TODO: 2.9. Complete the function body with your code (i.e., replace the pass statement).
        weights = [
            self._measurement_probability(measurements, particle) for particle in self._particles
        ]

        # Normalize the weights
        alphas = np.array(weights) / np.sum(weights)

        # Cumulative sum of the weights
        cumulative_sum = np.cumsum(alphas)
        # Determine the positions of the indices to resample
        positions = (np.arange(self._particle_count) + np.random.random()) / self._particle_count

        indexes = np.zeros(self._particle_count, "i")
        cumulative_index = 0
        for i, pos in enumerate(positions):
            while cumulative_sum[cumulative_index] < pos:
                cumulative_index += 1
            indexes[i] = cumulative_index

        # Resample the particles according to the indexes
        self._particles = self._particles[indexes]

    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(
                self._particles[:, 0],
                self._particles[:, 1],
                dx,
                dy,
                color="b",
                scale=15,
                scale_units="inches",
            )
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], "bo", markersize=1)

        return axes

    def show(
        self,
        title: str = "",
        orientation: bool = True,
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)

        axes.set_title(title + " (Iteration #" + str(self._iteration) + ")")
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", save_dir, self._timestamp)
            )

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = str(self._iteration).zfill(4) + " " + title.lower() + ".png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _init_particles(self, particle_count: int) -> np.ndarray:
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3*pi/2].

        Args:
            particle_count: Number of particles.

        Returns: A NumPy array of tuples (x, y, theta) [m, m, rad].

        """
        particles = np.empty((particle_count, 3), dtype=object)
        x_min, y_min, x_max, y_max = self._map.bounds()
        # TODO: 2.4. Complete the missing function body with your code.
        for i in range(particle_count):
            while True:
                x = np.random.uniform(x_min, x_max)
                y = np.random.uniform(y_min, y_max)
                if self._map.contains((x, y)):
                    break
            theta = np.random.choice([0, np.pi / 2, np.pi, 3 * np.pi / 2])
            # print(f"Particle {i}: ({x:.2f}, {y:.2f}, {theta:.2f})")
            particles[i] = (x, y, theta)
        return particles

    @staticmethod
    def _init_sensor_polar_coordinates(
        sensors: List[Tuple[float, float, float]],
    ) -> Tuple[List[float], List[float]]:
        """Converts the sensors' poses to polar coordinates wrt to the robot's coordinate frame.

        Args:
            sensors: Robot sensors location and orientation (x, y, theta) [m, m, rad].

        Return:
            ds: List of magnitudes [m].
            phi: List of angles [rad].

        """
        ds = [math.sqrt(sensor[0] ** 2 + sensor[1] ** 2) for sensor in sensors]
        phi = [math.atan2(sensor[1], sensor[0]) for sensor in sensors]

        return ds, phi

    def _sense(self, particle: Tuple[float, float, float]) -> List[float]:
        """Obtains the predicted measurement of every sensor given the robot's pose.

        Args:
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; inf if a sensor is out of range.

        """
        rays: List[List[Tuple[float, float]]] = self._sensor_rays(particle)
        z_hat: List[float] = []

        # TODO: 2.6. Complete the missing function body with your code.
        for ray in rays:
            # Compute the intersection of the ray with the map.
            intersection, distance = self._map.check_collision(ray, True)
            # Compute the distance to the intersection point
            if distance > self._sensor_range:
                z_hat.append(float("inf"))
            else:
                z_hat.append(distance)
        return z_hat

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian value.

        """
        # TODO: 2.7. Complete the function body (i.e., replace the code below).
        return math.exp(-0.5 * ((x - mu) / sigma) ** 2)

    def _measurement_probability(
        self, measurements: List[float], particle: Tuple[float, float, float]
    ) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with
        1.25 times the sensor range to perform the computation. This value has experimentally been
        proven valid to deal with missing measurements. Nevertheless, it might not be the optimal
        replacement value.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns:
            float: Probability.

        """
        probability = 1.0

        # TODO: 2.8. Complete the missing function body with your code.
        # Predicted measurement
        z_hat = self._sense(particle)
        # self._logger.info(f"z_hat:{z_hat}")
        measurements = [
            1.25 * self._sensor_range if z > self._sensor_range else z for z in measurements
        ]
        z_hat  = [
            1.25 * self._sensor_range if z > self._sensor_range else z for z in z_hat
        ]
                
        # Compare the predicted with the measure
        for z, z_hat_i in zip(measurements, z_hat):
            probability *= self._gaussian(z_hat_i, self._sigma_z, z)
        return probability

    def _sensor_rays(self, particle: Tuple[float, float, float]) -> List[List[Tuple[float, float]]]:
        """Determines the simulated sensor ray segments for a given particle.

        Args:
            particle: Particle pose (x, y, theta) in [m] and [rad].

        Returns: Ray segments. Format:
                 [[(x0_begin, y0_begin), (x0_end, y0_end)],
                  [(x1_begin, y1_begin), (x1_end, y1_end)],
                  ...]

        """
        x = particle[0]
        y = particle[1]
        theta = particle[2]

        # Convert sensors to world coordinates
        xw = [x + ds * math.cos(theta + phi) for ds, phi in zip(self._ds, self._phi)]
        yw = [y + ds * math.sin(theta + phi) for ds, phi in zip(self._ds, self._phi)]
        tw = [sensor[2] for sensor in self._sensors]

        rays = []

        for xs, ys, ts in zip(xw, yw, tw):
            x_end = xs + self._sensor_range * math.cos(theta + ts)
            y_end = ys + self._sensor_range * math.sin(theta + ts)
            rays.append([(xs, ys), (x_end, y_end)])

        return rays
