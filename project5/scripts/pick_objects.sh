#!/bin/sh
gnome-terminal --tab -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
gnome-terminal --tab -e  " roslaunch turtlebot_gazebo amcl_demo.launch" & 
sleep 5
gnome-terminal --tab -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
gnome-terminal --tab -e  " rosrun pick_objects pick_objects"