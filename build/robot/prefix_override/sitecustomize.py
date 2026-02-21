import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aclab/Desktop/workspaces/RobotROS2/ros2_ws/src/install/robot'
