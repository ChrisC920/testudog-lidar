#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

package_name = 'my_robot_controller'

class MyNode(Node):
    def __init__(self):
        super().__init__('my_robot_controller')
        self.timer = self.create_timer(1, self.output)

    def output(self):
        self.get_logger().info('Hello ROS2')
        

def main(args=None):
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
