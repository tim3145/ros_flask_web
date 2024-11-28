import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from flask_socketio import SocketIO

class ROS2BridgeNode(Node):
    def __init__(self, socketio):
        super().__init__('ros2_bridge_node')
        self.socketio = socketio
        self.publisher_ = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            'amcl_pose',  # 로봇 위치 정보 토픽
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # 로봇의 위치 정보를 Flask 서버로 전달
        self.socketio.emit('robot_position', {'x': msg.pose.position.x, 'y': msg.pose.position.y})

    def send_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        self.publisher_.publish(msg)
