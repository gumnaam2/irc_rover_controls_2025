import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/siddhant/irc_rover_controls_2025/arm_control/install/ik_control'
