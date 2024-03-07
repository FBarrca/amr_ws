import rclpy
from rclpy.node import Node

from amr_msgs.msg import PoseStamped, RangeScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path

import math
from transforms3d.euler import quat2euler

from amr_control.pure_pursuit import PurePursuit
import message_filters
from amr_msgs.msg import PoseStamped, RangeScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class PurePursuitNode(Node):
    def __init__(self):
        """Pure pursuit node initializer."""
        super().__init__("pure_pursuit")

        # Parameters
        self.previous_distance = 0.1
        self.crashed = False
        self.declare_parameter("dt", 0.05)
        dt = self.get_parameter("dt").get_parameter_value().double_value

        self.declare_parameter("lookahead_distance", 0.5)
        lookahead_distance = (
            self.get_parameter("lookahead_distance").get_parameter_value().double_value
        )

        # Subscribers
        # self._subscriber_pose = self.create_subscription(
        #     PoseStamped, "/pose", self._compute_commands_callback, 10
        # )
        self._subscribers: list[message_filters.Subscriber] = []
        self._subscribers.append(message_filters.Subscriber(self, Odometry, "odom"))
        self._subscribers.append(message_filters.Subscriber(self, RangeScan, "us_scan"))
        self._subscribers.append(message_filters.Subscriber(self, PoseStamped, "pose"))
        ts = message_filters.ApproximateTimeSynchronizer(self._subscribers, queue_size=10, slop=10)
        ts.registerCallback(self._compute_commands_callback)
        self._subscriber_path = self.create_subscription(Path, "path", self._path_callback, 10)

        # Publishers
        self._publisher = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self._pose_publisher = self.create_publisher(PoseStamped, "pose_recalc", 10)
        # Attribute and object initializations
        self._pure_pursuit = PurePursuit(dt, lookahead_distance)

    def _compute_commands_callback(self, odom_msg: Odometry, us_msg: RangeScan, pose_msg: PoseStamped):
            """Subscriber callback. Executes a pure pursuit controller and publishes v and w commands.

            Starts to operate once the robot is localized.

            Args:
                pose_msg: Message containing the estimated robot pose.

            """
            if pose_msg.localized:
                # Parse pose
                x = pose_msg.pose.position.x
                y = pose_msg.pose.position.y
                quat_w = pose_msg.pose.orientation.w
                quat_x = pose_msg.pose.orientation.x
                quat_y = pose_msg.pose.orientation.y
                quat_z = pose_msg.pose.orientation.z
                _, _, theta = quat2euler((quat_w, quat_x, quat_y, quat_z))
                theta %= 2 * math.pi
                # Parse odometry vel
                v = odom_msg.twist.twist.linear.x
                # Parse ultrasonic sensor readings
                measurements = us_msg.ranges
                
                
                #previous distance
                front_distance = min(us_msg.ranges[2],us_msg.ranges[3],us_msg.ranges[4],us_msg.ranges[5])
                
                difference_distance = abs(front_distance-self.previous_distance)
                
                if difference_distance < 0.01 and front_distance < 0.2:
                    self.crashed = True 
                if self.crashed and front_distance > 0.35:
                    self.crashed = False
                    # send message telling to localize again!
                    self.publish_not_localized()
                    
                #See if crashed
                self.previous_distance = front_distance
                
                # Execute pure pursuit
                v, w = self._pure_pursuit.compute_commands(x, y, theta, self._logger, measurements,self.crashed)
                self.get_logger().info(f"Commands: v = {v:.3f} m/s, w = {w:+.3f} rad/s")

                # Publish
                self._publish_velocity_commands(v, w)

    def _path_callback(self, path_msg: Path):
        """Subscriber callback. Saves the path the pure pursuit controller has to follow.

        Args:
            path_msg: Message containing the (smoothed) path.

        """
        # TODO: 4.1. Complete the function body with your code (i.e., replace the pass statement).
        # Get the x and y coordinates of the path
        path = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
        self._pure_pursuit.path = path
        

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """Publishes velocity commands in a geometry_msgs.msg.TwistStamped message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].

        """
        msg = TwistStamped()
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self._publisher.publish(msg)
    def publish_not_localized(self):
        """Publishes the robot's pose estimate in a custom amr_msgs.msg.PoseStamped message.

        Args:
            x_h: x coordinate estimate [m].
            y_h: y coordinate estimate [m].
            theta_h: Heading estimate [rad].

        """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.localized = False
        self._pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()

    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pass

    pure_pursuit_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
