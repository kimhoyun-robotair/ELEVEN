import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hoyunkim/amr_isaac51/ros2_ws/install/amr_tools'
