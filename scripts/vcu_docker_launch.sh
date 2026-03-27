#!/bin/bash
echo "Starting docker early launch"
source /home/devuser/.bashrc
source /opt/ros/jazzy/setup.bash
source /home/devuser/vcu_ws/install/setup.bash
tmux start-server
tmux new-session -d
sleep 5
tmux new-window 'ros2 launch vcu_launch vcu.launch.py'

sleep 10 # Sleep is important, take care of yourselves -> with 5sec it does not init can and ethernet properly
tmux new-window 'ros2 launch vcu_launch vcu-smallRecording.launch'
sleep 5
tmux new-window 'ros2 launch vcu_launch vcu-foxglove.launch.py'


echo "Finishing docker early launch"
