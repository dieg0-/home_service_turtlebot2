#!/bin/sh
xterm -e "source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find home_service_robot_configs)/worlds/apartment_and_tree.world " &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find home_service_robot_configs)/maps/my_gmapped_apartment.yaml" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects_node"
