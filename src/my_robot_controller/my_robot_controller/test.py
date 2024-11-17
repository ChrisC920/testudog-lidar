import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

class NodeSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        
        self.sensor_subscriber = self.create_subscription(LaserScan, "/scan", self.sensor_callback, 10)

    def sensor_callback(self, msg:LaserScan):
        self.get_logger().info(f"Received a message: {msg}")

def main(args=None):
    rclpy.init()
    node = NodeSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
