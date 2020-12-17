#!/bin/bash
# Shell script associated with starting up the hide and seek demonstration.
# Ensure the Control + C kills all of the associated processes.
trap ctrl_c INT

function ctrl_c() {
     echo "Killing Gym";
     killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient
}

# Kill any remaining processes from previous runs prior to starting this one.
killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient
# Copy over the environment files.
cp /root/catkin-ws/gym-gazebo-hide-and-seek/gym_gazebo/envs /usr/local/lib/python2.7/dist-packages/gym_gazebo-0.0.2-py2.7.egg/gym_gazebo/ -R
# Gazebo display setup.
export GAZEBO_MASTER_URI=http://localhost:10100
export GAZEBO_MODEL_PATH=/root/catkin-ws/gym-gazebo-hide-and-seek/gym_gazebo/envs/assets/models
# Kick off the main script.
vglrun python maze_multi_turtlebot_lidar_qlearn.py
