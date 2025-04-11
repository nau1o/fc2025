import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/naynn/Documents/fc2025/fc2025/install/node_test'
