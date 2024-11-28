import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MoveBaseNode(Node):
    def __init__(self):
        super().__init__('move_base_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)

    def send_goal(self, goal):
        self.publisher_.publish(goal)
