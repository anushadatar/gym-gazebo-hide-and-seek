#!/bin/bash
# Helper script for viewing both hider and seeker camera output.
# Note that turtle 1 is the seeker and turtle 2 is the hider.
export ROS_MASTER_URI=http://localhost:10099
rosrun image_view image_view image:=/turtle_1/camera/rgb/image_raw & rosrun image_view image_view image:=/turtle_2/camera/rgb/image_raw
