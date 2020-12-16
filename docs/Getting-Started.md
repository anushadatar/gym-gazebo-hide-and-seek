# Getting Started
Learning how to set up a working system using gym, gazebo, and ROS was a major component of our project.
We eventually landed on using the (now-deprecated) [gym-gazebo](https://github.com/erlerobot/gym-gazebo) as a starting point, 
and we made a variety of modifications to its workflow to get our version running. For more information about this process,
check out our [blog posts](https://anushadatar.github.io/gym-gazebo-hide-and-seek/Blog-Posts.html). This page provides some
context into how to recreate our setup.

Note that this setup uniquely applies to a Docker container running Ubuntu 16.04 and ROS Kinetic, and it will 
likely require modification on other platforms. We also created this through our own troubleshooting
errors as we tried to set this up, so there are likely more efficient installation methods.

## Initial Setup
First, clone the repository install the associated python dependencies.
```
git clone https://github.com/anushadatar/gym-gazebo-hide-and-seek.git
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.8 python3-pip
sudo pip install -e .
sudo apt-get upgrade
https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python2.7 get-pip.py --force-reinstall
pip install scikit-image
cd gym_gazebo/envs/installation
```
## Add ROS Dependencies 
### Configure intel realsense to work with turtlebot.
```
Based off of https://github.com/IntelRealSense/librealsense/issues/4781
# Set up keys and update container/
sudo apt-get update
sudo apt-get install -y lsb-release
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
sudo apt-get update

# Manually install necessary dependencies.
sudo apt-get install -y binutils cpp cpp-5 dkms fakeroot gcc gcc-5 kmod libasan2 libatomic1 libc-dev-bin libc6-dev libcc1-0 libcilkrts5 libfakeroot libgcc-5-dev libgmp10 libgomp1 libisl15 libitm1 liblsan0 libmpc3 libmpfr4 libmpx0 libquadmath0 libssl-dev libssl-doc libtsan0 libubsan0 libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic linux-libc-dev make manpages manpages-dev menu patch zlib1g-dev
sudo apt-get install -y libssl-dev libssl-doc libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic zlib1g-dev

# Downloadd and unpack the package, replace the script.
apt-get download ros-kinetic-librealsense
dpkg-deb -R ros-kinetic-librealsense*.deb ros-rslib/

wget https://gist.githubusercontent.com/dizz/404ef259a15e1410d692792da0c27a47/raw/3769e80a051b5f2ce2a08d4ee6f79c766724f495/postinst
chmod +x postinst
cp postinst ros-rslib/DEBIAN
dpkg-deb -b ./ros-rslib/ ros-kinetic-librealsense_1.12.1-0xenial-20190830_icrlab_amd64.deb

# Install the version that works with the container
dpkg -i ros-kinetic-librealsense_1.12.1-0xenial-20190830_icrlab_amd64.deb

# Lock from updates
apt-mark hold ros-kinetic-librealsense
```
## Turtlebot setup
```
cd ~/gym-gazebo-hide-and-seek/gym-gazebo/envs/catkin_ws
git clone https://github.com/turtlebot/turtlebot_simulator.git
pip install -r requirements.txt .
git clone https://github.com/yujinrobot/kobuki.git
cd ..
bash turtlebot_setup.bash
```
## Set up gym 
### Manually copy the assets to your local installation.
``` 
cd ~/gym-gazebo-hide-and-seek/gym-gazebo
source /opt/ros/kinetic/setup.bash
cp ~/gym-gazebo-hide-and-seek/gym-gazebo/envs/assets /PATH_TO_LOCAL_GYM_GAZEBO_INSTALLATION/envs/ -r
```
### Manually resolve python 2 dependency.
```
vim YOUR_PYTHON_INSTALLATION/gym/wrappers/monitor.py
```
Omit the `exist_ok` parameter on line 69 of this file.
## Run an example test script 
```
cd gym_gazebo/envs/installation
bash turtlebot_setup.bash
cd ~/gym-gazebo-hide-and-seek/examples/turtlebot
vglrun python circuit2_turtlebot_lidar_qlearn.py
```
Note the `GAZEBO_MASTER_URI` variable from the script's output. In another terminal, set this URI manually and then
start Gazebo.
```
export GAZEBO_MASTER_URI=URI_GOES_HERE
vglrun gzclient
```
