# ROS2 Perception Module

ROS2 기반 객체 인식(YOLOv8) 및 거리 측정 모듈입니다.

## 1. 설치 방법

```bash
# 1. 워크스페이스로 이동
cd ~/ai_robot_9/src

# 2. 파이썬 라이브러리 설치
pip install -r perception/requirements.txt

requirements.txt
------------------
ultralytics
opencv-python
roboflow
numpy<2.0
----------------

## 2. 사용법

```bash
cd ~/ai_robot_9
ros2 run perception perception_node
ros2 launch module_test interface_viewer.launch.py
하면 됩니다.

## 3. 사용한 툴

# 1. roboflow
https://app.roboflow.com/join/eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ3b3Jrc3BhY2VJZCI6ImNzVVdVWEoyZFlhWFlQVmtnaVpTeHg2eWZUdDIiLCJyb2xlIjoib3duZXIiLCJpbnZpdGVyIjoia3JrcjEyMDNAc251LmFjLmtyIiwiaWF0IjoxNzY0NzcyNDYzfQ.uqDREufhNMH_Az8x5yREqpxxj_cwIQS9A0owbUgQO7A
# 2. google colab
https://colab.research.google.com/drive/1VrBfWyv0UyfIMjkT6N54ZKkTt294NYca?usp=sharing
