#!/bin/sh
GO_TO_WORKSPACE="cd $(dirname "$0"); "
ROS_SOURCE_CMD="${GO_TO_WORKSPACE} source /opt/ros/kinetic/setup.bash; source ../../devel/setup.bash; "

terminator -e "${ROS_SOURCE_CMD} roslaunch my_robot world.launch" &
sleep 5
terminator -e "${ROS_SOURCE_CMD} roslaunch my_robot amcl.launch" &
sleep 5
terminator -e "${ROS_SOURCE_CMD} roslaunch my_robot view_navigation.launch" &
sleep 5
terminator -e "${ROS_SOURCE_CMD} rosrun pick_objects pick_objects" &