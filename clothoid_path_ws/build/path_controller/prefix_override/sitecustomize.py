import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rituraj/Northeastern_University/MR/final_project/clothoid_path_ws/install/path_controller'
