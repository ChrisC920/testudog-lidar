import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import random

package_name = 'my_robot_controller'

class SLAMTestPublisher(Node):
    def __init__(self):
        super().__init__('slam_test_publisher')

        # Publishers for LaserScan and Odometry
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster for base_link to odom
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers for periodic publishing
        self.scan_timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        self.odom_timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz

        # Robot state variables
        self.current_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.1  # rad/s

    def publish_scan(self):
        """Simulates 2D lidar data."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180  # 1-degree increments
        scan.range_min = 0.1
        scan.range_max = 10.0

        # Generate fake laser ranges (e.g., a circular room)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = []
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            # Simulate walls at 5m with some random noise
            range_val = 5.0 + random.uniform(-0.1, 0.1)
            if random.random() < 0.05:  # Add some noise or dropouts
                range_val = float('inf')
            ranges.append(range_val)

        scan.ranges = ranges
        self.laser_pub.publish(scan)

    def publish_odom(self):
        """Simulates odometry data."""
        dt = 0.1  # Time step (10 Hz)
        self.current_time = self.get_clock().now()

        # Update robot position
        self.x += self.linear_speed * math.cos(self.theta) * dt
        self.y += self.linear_speed * math.sin(self.theta) * dt
        self.theta += self.angular_speed * dt

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.quaternion_from_yaw(self.theta)

        # Twist
        odom.twist.twist.linear.x = self.linear_speed
        odom.twist.twist.angular.z = self.angular_speed

        self.odom_pub.publish(odom)

        # Publish transform
        self.publish_tf()

    def publish_tf(self):
        """Publishes TF from odom to base_link."""
        t = TransformStamped()
        t.header.stamp = self.current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = self.quaternion_from_yaw(self.theta)
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def quaternion_from_yaw(yaw):
        """Helper to create a Quaternion from a yaw angle."""
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SLAMTestPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
