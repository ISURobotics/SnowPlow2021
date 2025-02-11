import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/snowclone/colcon_ws/src/SnowPlow2021/install/SnowPlow2021'
