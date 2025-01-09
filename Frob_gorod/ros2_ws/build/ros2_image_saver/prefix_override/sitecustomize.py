import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dark516/Documents/RTK_robot/Frob_gorod/ros2_ws/install/ros2_image_saver'
