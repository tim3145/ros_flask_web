#!/bin/bash

# ROS 2 작업 공간 소스
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# Flask 서버 및 ROS 2 브릿지 노드 실행
gnome-terminal -- bash -c "python3 bridge_node.py"
gnome-terminal -- bash -c "python3 app.py"
