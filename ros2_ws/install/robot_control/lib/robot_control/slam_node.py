import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)

    def publish_map(self, map_data):
        self.publisher_.publish(map_data)
