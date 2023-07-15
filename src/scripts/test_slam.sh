#!/bin/sh
GO_TO_WORKSPACE="cd $(dirname "$0"); "
ROS_SOURCE_CMD="${GO_TO_WORKSPACE} source /opt/ros/kinetic/setup.bash; source ../../devel/setup.bash; "

terminator -e "${ROS_SOURCE_CMD} roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
terminator -e "${ROS_SOURCE_CMD} roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
terminator -e "${ROS_SOURCE_CMD} roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
terminator -e "${ROS_SOURCE_CMD} roslaunch turtlebot_teleop keyboard_teleop.launch" &
