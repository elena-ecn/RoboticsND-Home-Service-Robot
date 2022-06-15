#!/bin/sh

# Deploy turtlebot3 in custom Gazebo world
xterm  -e  " roslaunch home_service home_service_world.launch " &
sleep 15

# Launch gmapping with tuned parameters to create a 2D occupancy grid map
xterm  -e  " roslaunch home_service gmapping_demo.launch " &
sleep 5

# Launch rviz with saved configurations 
xterm  -e " roslaunch home_service view_navigation.launch" &
sleep 5

# Launch teleop to teleoperate the robot and create a map 
xterm  -e  " rosrun teleop_twist_keyboard teleop_twist_keyboard.py " 
