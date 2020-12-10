---
layout: "default"
title: Blog Posts
---
# Blog Posts

## Project Update: 12/9
One of the major sets of challenges we have faced over the course of this project involves configuring the OpenAI gym reinforcement learning framework to work with ROS and gazebo. Our last blog posts detailed some of our decisions and processes around selecting which external frameworks to use and how to incorporate them, and this blog post will detail how we configured the gym-gazebo library to work with our setup.

The gym-gazebo environment, as-is, worked with an earlier version of python 2, ROS kinetic, Initially, we tried to set up the gym-gazebo environment on our own personal machines. Our first pass involved trying to update the necessary elements of the [gym-gazebo source](https://github.com/erlerobot/gym-gazebo/) to ROS noetic (using documents like this [migration guide](http://wiki.ros.org/noetic/Migration) as resources). Then, we tried using separate environments that could support ROS kinetic, like separate virtual machines or Docker environments. While we saw some limited success in the Docker environment, we soon ran into resource constraints. 
 
At that point, we (with Paul’s help!) got access to deepthought and were able to run a docker container there that includes Ubuntu 16.04, ROS Kinetic, Python 2, and just enough Python 3 for the library dependencies to work as expected.
 
Once we had access to this container, we configured the container environment to allow us to use the gym-gazebo framework to run a simple maze example.

Note that because this library has been archived for quite a while, we had to significantly deviate from the expected setup workflow and make some manual changes to the environment and library.
 
After those changes, we were able to train the turtlebot to solve the maze. 
Below are the steps we developed for setting up the gym-gazebo framework:
- Connect to the VPN, and navigate to http://deepthought.olin.edu:40001/vnc.html
- Create a catkin workspace
- Clone erlerobotics/gym-gazebo in src
- Run setup_kinetic.bash
- Change directory into an inner catkin workspace inside gym-gazebo
- Clone turtlebot_simulator and kobuki repos
- Run apt update/upgrade flow
- From gym-gazebo root run
  - pip install requirements.txt 
  - python setup.py install
- Remaking the catkin workspace + pip installing scikit-image and additional libraries may be required
- Run this [manual fix](https://github.com/IntelRealSense/librealsense/issues/4781) to configure the librealsense dependencies needed for turtlebot. This fix allows
for the installation of the specific dependency that cans support turtlebot on Python 2.3
- Run sudo apt-get install ros-kinetic-turtlebot*
- Run turtlebot_setup.bash
- Run export GAZEBO_MODEL_PATH=~/catkin_ws/src/gym-gazebo/gym-gazebo/envs/assets/models/
- Run an example turtlebot script
- Edit out the erroring line in gym, exclude ‘exists_ok’ from “os.makedirs(...exists_ok=True)”
  - Specifically, we changed line 69 in /usr/local/lib/python2.7/dist_packages/gym/wrappers/monitor.py to just os.makedirs(directory)
- Rerun the script, record the GAZEBO_MASTER_URL from the logs
- Start gazebo from another terminal by running `export GAZEBO_MASTER_URL={url from before}` and the `vglrun gzclient`
