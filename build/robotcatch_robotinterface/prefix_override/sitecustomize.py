import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/david-linux-arm/linux/ros2_ws/install/robotcatch_robotinterface'
