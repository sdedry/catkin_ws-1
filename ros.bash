#!/bin/bash
SESSION=$USER

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:1 -n 'ROS'
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux select-pane -t 1
tmux send-keys "sleep 5" C-m
tmux send-keys "rosrun navio2_imu_pub imu_pub" C-m
tmux split-window -v
tmux send-keys "sleep 5" C-m
tmux send-keys "rostopic echo mag_readings" C-m
tmux select-pane -t 0
tmux split-window -v
tmux send-keys "sleep 5"
tmux send-keys "rostopic echo imu_readings" C-m


# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
