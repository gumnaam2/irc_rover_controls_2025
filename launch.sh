#!/bin/bash
SSH_ID=$1
terminator --config config \
    -e "bash -c 'echo \"can node\"; ssh -t $SSH_ID \"ros2 launch joy_can LaunchAll; bash;\"'" &
#terminator --no-dbus --config .config/terminator/config  -e "bash -c 'echo \"Hello \"; bash'" &