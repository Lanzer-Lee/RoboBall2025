import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lanzer/robocup/basketball_ws/install/sensor_pkg'
