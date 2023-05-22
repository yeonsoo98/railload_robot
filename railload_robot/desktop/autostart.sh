#!/bin/bash
mkdir -p /home/"$USER"/.config/autostart
echo "autostart directory is created"

cp /home/"$USER"/ros2_ws/src/railload_robot/railload_robot/desktop/robot.desktop /home/"$USER"/.config/autostart/robot.desktop
echo "robot.desktop is copied to autostart directory"
