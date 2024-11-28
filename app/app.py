from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

app = Flask(__name__)
socketio = SocketIO(app)

# ROS 2 Bridge Node
class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_bridge_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            'amcl_pose',  # 로봇 위치 정보 토픽
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        # 로봇 위치 정보 수신 (맵을 기준으로 현재 위치 업데이트)
        socketio.emit('robot_position', {'x': msg.pose.position.x, 'y': msg.pose.position.y})

    def send_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        self.publisher_.publish(msg)

# 웹소켓에서 로봇 목표 위치를 받으면 해당 위치로 이동
@socketio.on('move_robot')
def handle_move_robot(data):
    x = data['x']
    y = data['y']
    ros2_node.send_goal(x, y)

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    rclpy.init()
    ros2_node = ROS2BridgeNode()

    # Flask 서버 실행
    socketio.run(app, host='0.0.0.0', port=5000)
    rclpy.spin(ros2_node)
