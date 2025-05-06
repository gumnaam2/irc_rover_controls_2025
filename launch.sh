#!/bin/bash
SSH_ID=$1
terminator --config config \
    -e "bash -c 'echo \"hello\"; ssh -t $SSH_ID \"ros2 launch joy_can ArmDriveLaunch.py; bash;\"'" &
#terminator --no-dbus --config .config/terminator/config  -e "bash -c 'echo \"Hello \"; bash'" &