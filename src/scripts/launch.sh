#!/bin/sh
GO_TO_WORKSPACE="cd $(dirname "$0"); "
ROS_SOURCE_CMD="${GO_TO_WORKSPACE} source /opt/ros/kinetic/setup.bash; source ../../devel/setup.bash; "

terminator -e " gazebo " &
sleep 5
terminator -e "${ROS_SOURCE_CMD} roscore" & 
sleep 5
terminator -e "${ROS_SOURCE_CMD} rosrun rviz rviz" 