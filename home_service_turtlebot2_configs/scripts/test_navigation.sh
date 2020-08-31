#!/bin/sh
xterm -e "source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find home_service_turtlebot2_configs)/worlds/apartment_and_tree.world " &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find home_service_turtlebot2_configs)/maps/my_gmapped_apartment.yaml" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"
