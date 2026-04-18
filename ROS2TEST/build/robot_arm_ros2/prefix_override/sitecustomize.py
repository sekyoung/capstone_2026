import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sek_kim/Desktop/QP/ROS2TEST/install/robot_arm_ros2'
