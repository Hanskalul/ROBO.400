import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from math import pow, atan2, asin, sqrt, pi, copysign, degrees

class TurtleBot(Node):

    def __init__(self):
        # Creates a node with name 'turtlebot_controller'.
        super().__init__('turtlebot_controller')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = self.create_subscription(
            Odometry, '/odom', self.update_pose, 10)

        self.pose = Odometry()

        # Timer replaces the ROS1 rate-limited while loop; called at 10 Hz.
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = None

        self.side_length = 0.0
        self.i = 0
        self.goal_pose = Pose()
        self.distance_tolerance = 0.0

    import math

    def quaternion_to_euler_degrees(self, x, y, z, w):
        # Roll (x-axis rotation)
        sin_r = 2.0 * (w * x + y * z)
        cos_r = 1.0 - 2.0 * (x * x + y * y)
        roll = atan2(sin_r, cos_r)
        
        # Pitch (y-axis rotation)
        sin_p = 2.0 * (w * y - z * x)
        if abs(sin_p) >= 1.0:
            pitch = copysign(pi / 2, sin_p)
        else:
            pitch = asin(sin_p)
            
        # Yaw/Heading (z-axis rotation)
        sin_y = 2.0 * (w * z + x * y)
        cos_y = 1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(sin_y, cos_y)
        
        # Convert radians to degrees
        # return degrees(roll), degrees(pitch), degrees(yaw)
        return roll, pitch, yaw


    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.y - self.pose.pose.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        cmd_vel = constant * self.euclidean_distance(goal_pose)

        if cmd_vel < -0.11:
            cmd_vel = -0.11
        elif cmd_vel > 0.11:
            cmd_vel = 0.11
            
        return cmd_vel

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.pose.pose.position.y, goal_pose.x - self.pose.pose.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        r, p, theta = self.quaternion_to_euler_degrees(self.pose.pose.pose.orientation.x,
                                                       self.pose.pose.pose.orientation.y,
                                                       self.pose.pose.pose.orientation.z,
                                                       self.pose.pose.pose.orientation.w)
        cmd_vel = constant * (self.steering_angle(goal_pose) - theta)

        if cmd_vel < -1.0:
            cmd_vel = -1.0
        elif cmd_vel > 1.0:
            cmd_vel = 1.0

        return cmd_vel

    def move2goal(self):
        """Sets up the goal and starts the timer-driven control loop."""
        # Get the input from the user.
        self.side_length = float(input("Set length of sides: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        self.distance_tolerance = float(input("Set your tolerance: "))

        # In ROS2, blocking loops are replaced with a timer callback that
        # rclpy.spin() drives; this keeps the node responsive to callbacks
        # (e.g. Ctrl+C / shutdown) instead of busy-waiting.
        self.goal_pose.x = 0.0
        self.goal_pose.y = 0.0
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def control_loop(self):
        """Timer callback: proportional controller step toward the goal."""
        vel_msg = TwistStamped()

        if self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:
            # Proportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.twist.linear.x = self.linear_vel(self.goal_pose)
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.twist.angular.x = 0.0
            vel_msg.twist.angular.y = 0.0
            vel_msg.twist.angular.z = self.angular_vel(self.goal_pose)

            self.velocity_publisher.publish(vel_msg)
        else:
            # Goal reached: stop the robot and cancel the timer.
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.angular.z = 0.0
            self.get_logger().info(f"Corner {self.i} reached")

            if self.i == 0:
                self.goal_pose.x = self.side_length
                self.goal_pose.y = 0.0
                self.i += 1
            elif self.i == 1:
                self.goal_pose.x = self.side_length
                self.goal_pose.y = self.side_length
                self.i += 1
            elif self.i == 2:
                self.goal_pose.x = 0.0
                self.goal_pose.y = self.side_length
                self.i += 1
            elif self.i == 3:
                self.goal_pose.x = 0.0
                self.goal_pose.y = 0.0
                self.i += 1
            elif self.i == 4:
                self.velocity_publisher.publish(vel_msg)
                self.timer.cancel()
                self.get_logger().info('Goal reached.')


def main(args=None):
    rclpy.init(args=args)
    x = TurtleBot()
    try:
        x.move2goal()
        rclpy.spin(x)
    except KeyboardInterrupt:
        pass
    finally:
        x.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()