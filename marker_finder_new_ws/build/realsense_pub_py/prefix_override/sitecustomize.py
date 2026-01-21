import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ryleighbyrne/marker_finder_new_ws/install/realsense_pub_py'
