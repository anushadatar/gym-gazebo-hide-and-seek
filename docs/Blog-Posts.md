---
layout: "default"
title: Blog Posts
---
# Blog Posts

## Project Update 12/9: Setting Up the gym-gazebo environment
One of the major sets of challenges we have faced over the course of this project involves configuring the OpenAI gym reinforcement learning framework to work with ROS and gazebo. Our last blog posts detailed some of our decisions and processes around selecting which external frameworks to use and how to incorporate them, and this blog post will detail how we configured the gym-gazebo library to work with our setup.

The gym-gazebo environment, as-is, worked with an earlier version of python 2, ROS kinetic, Initially, we tried to set up the gym-gazebo environment on our own personal machines. Our first pass involved trying to update the necessary elements of the [gym-gazebo source](https://github.com/erlerobot/gym-gazebo/) to ROS noetic (using documents like this [migration guide](http://wiki.ros.org/noetic/Migration) as resources). Then, we tried using separate environments that could support ROS kinetic, like separate virtual machines or Docker environments. While we saw some limited success in the Docker environment, we soon ran into resource constraints. 
 
At that point, we (with Paul’s help!) got access to deepthought and were able to run a docker container there that includes Ubuntu 16.04, ROS Kinetic, Python 2, and just enough Python 3 for the library dependencies to work as expected. While we faced a few roadblocks.
 
Once we had access to this container, we configured the container environment to allow us to use the gym-gazebo framework to run a simple maze example.

Note that because this library has been archived for quite a while, we had to significantly deviate from the expected setup workflow and make some manual changes to the environment and library.
 
After those changes, we were able to train the turtlebot to solve the maze. 
Below are the steps we the steps we roughly developed for setting up the gym-gazebo framework:
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
We'll verify that these are the exact steps necessary to set up the environment and publish them on the [Getting Started](https://anushadatar.github.io/gym-gazebo-hide-and-seek/Getting-Started.html) page of the project site.

# Project Update 11/23: Selecting a framework
One of our primary learning goals for this project was to practice integrating existing machine learning algorithms into larger robotics frameworks. There are two reasons we chose this -  to get to learn more about specific machine learning algorithms (specifically in reinforcement learning) and to get to practice integrating multiple complex systems into a cohesive project. As a result, we are trying to use OpenAI’s gym framework with ROS and Gazebo. Unsurprisingly, getting ROS, Gazebo, and OpenAI’s gym to talk to each other has been really challenging.

A couple issues we’ve been running into as of our last attempt at getting the RL setup with Gazebo off the ground include setting up a compatible ROS distribution and OpenCV errors firing with the wrong version of Python. We expect to be able to smooth out these errors in the coming week, and start running iterations of our hide-and-seek simulation, particularly using resources from The Construct, an online robotics course which appears to have significant material on connecting OpenAI’s Gym with Gazebo for the OpenAI package approach and a docker container for the gym-gazebo approach. 

There are two paths that we are considering at the moment.
## OpenAI ROS package
There is an (old) [OpenAI ROS package](http://wiki.ros.org/openai_ros) to support individual sets of task, robot, and gazebo environments. The gazebo environment generally just serves to connect the simulation to Gazebo. The robot environment inherits from the Gazebo environment to support 

While the framework makes sense as a concept, we are running into a compatiblity issues on a variety of platforms, and were unable to write our own examples or get existing examples running, including the [Turtlebot2 example](http://wiki.ros.org/openai_ros/TurtleBot2%20with%20openai_ros) and the tutorials from the construct [The Construct](https://www.theconstructsim.com/using-openai-ros/). A lot of these issues seemed to mainly be due to incompatibilities between dependencies that we cannot yet overcome, even with manual fixes and dramatically changing our own development environments.

## erlerobot gym-gazebo framework
This now-deprecated library seems to have the bindings necessary to run gym using ROS and gazebo. We initially tried running this using the setup we have for this class (ROS Noetic and Ubuntu 20.04). While we were able to make some progress with this approach, it ultimately proved too limiting as there were a variety of Python 2 and ROS Kinetic dependencies the newest version of Ubuntu simply could not support. As a result, we pivoted to using a docker container with a custom launch script that allowed to us to use Docker for the development environment but use our local gazebo installation. We are facing roadblocks here with dependencies as well that we are actively investigating.
