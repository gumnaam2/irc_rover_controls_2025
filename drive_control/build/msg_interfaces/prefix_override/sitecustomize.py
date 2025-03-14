import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sudhindra/irc_2025/drive_control/install/msg_interfaces'
