import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jimmy/ros2_ws/src/install/bootcamp_turtle_mover'
