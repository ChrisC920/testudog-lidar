import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf2_ros
import math

class LaserScanPublisher(Node):

    def __init__(self):
        super().__init__('laser_scan_publisher')

        # Remove LaserScan publisher
        # self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)

        # Create a TransformBroadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Set a timer to publish Odometry and transform
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize position and angle
        self.angle = 0.0

    def timer_callback(self):
        # Remove LaserScan message creation and publication
        # scan = LaserScan()
        # scan.header.stamp = self.get_clock().now().to_msg()
        # scan.header.frame_id = 'laser'
        # scan.angle_min = -np.pi / 4
        # scan.angle_max = np.pi / 4
        # scan.angle_increment = np.pi / 180 
        # scan.time_increment = 0.0
        # scan.range_min = 0.1
        # scan.range_max = 10.0
        # ranges = []
        # for i in range(181):
        #     angle = scan.angle_min + i * scan.angle_increment
        #     if -np.pi / 8 <= angle <= np.pi / 8:
        #         ranges.append(5.0)  # Front side of the box
        #     elif angle < -np.pi / 8:
        #         ranges.append(3.0)  # Left side of the box
        #     else:
        #         ranges.append(3.0)  # Right side of the box
        # scan.ranges = ranges
        # scan.intensities = [0.0] * 181
        # self.publisher_.publish(scan)

        # Create an Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Update position and orientation for circular movement
        radius = 1.0  # Radius of the circle
        self.angle += 0.1  # Increment angle

        odom.pose.pose.position.x = radius * math.cos(self.angle)
        odom.pose.pose.position.y = radius * math.sin(self.angle)
        odom.pose.pose.position.z = 0.0

        # Update orientation to face the direction of movement
        odom.pose.pose.orientation.z = math.sin(self.angle / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.angle / 2.0)

        odom.twist.twist.linear.x = 0.1  # Linear velocity in x direction
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.1  # Angular velocity

        # Publish Odometry
        self.odom_publisher_.publish(odom)

        # Create and publish the transform from map to odom
        map_to_odom_transform = TransformStamped()
        map_to_odom_transform.header.stamp = self.get_clock().now().to_msg()
        map_to_odom_transform.header.frame_id = 'map'
        map_to_odom_transform.child_frame_id = 'odom'
        map_to_odom_transform.transform.translation.x = 0.0
        map_to_odom_transform.transform.translation.y = 0.0
        map_to_odom_transform.transform.translation.z = 0.0
        map_to_odom_transform.transform.rotation.x = 0.0
        map_to_odom_transform.transform.rotation.y = 0.0
        map_to_odom_transform.transform.rotation.z = 0.0
        map_to_odom_transform.transform.rotation.w = 1.0

        # Broadcast the transform from map to odom
        self.tf_broadcaster.sendTransform(map_to_odom_transform)

        # Create and publish the transform from odom to base_link
        odom_to_base_link_transform = TransformStamped()
        odom_to_base_link_transform.header.stamp = self.get_clock().now().to_msg()
        odom_to_base_link_transform.header.frame_id = 'odom'
        odom_to_base_link_transform.child_frame_id = 'base_link'
        odom_to_base_link_transform.transform.translation.x = odom.pose.pose.position.x
        odom_to_base_link_transform.transform.translation.y = odom.pose.pose.position.y
        odom_to_base_link_transform.transform.translation.z = 0.0
        odom_to_base_link_transform.transform.rotation = odom.pose.pose.orientation

        # Broadcast the transform from odom to base_link
        self.tf_broadcaster.sendTransform(odom_to_base_link_transform)

        # Create and publish the transform from base_link to imu_link
        base_link_to_imu_link_transform = TransformStamped()
        base_link_to_imu_link_transform.header.stamp = self.get_clock().now().to_msg()
        base_linkt_to_imu_link_transform.header.frame_id = 'base_link'
        base_link_to_imu_link_transform.child_frame_id = 'imu_link'
        base_link_to_imu_link_transform.transform.translation.x = 0.0
        base_link_to_imu_link_transform.transform.translation.y = 0.0
        base_link_to_imu_link_transform.transform.translation.z = 0.0
        base_link_to_imu_link_transform.transform.rotation.x = 0.0
        base_link_to_imu_link_transform.transform.rotation.y = 0.0
        base_link_to_imu_link_transform.transform.rotation.z = 0.0
        base_link_to_imu_link_transform.transform.rotation.w = 1.0

        # Broadcast the transform from base_link to imu_link
        self.tf_broadcaster.sendTransform(base_link_to_imu_link_transform)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_publisher = LaserScanPublisher()
    rclpy.spin(laser_scan_publisher)

    laser_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()