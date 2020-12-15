import gym
import rospy
import roslaunch
import time
import numpy as np
import cv2
import sys
import os
import random

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from gym.utils import seeding
from cv_bridge import CvBridge, CvBridgeError

import skimage as skimage
from skimage import transform, color, exposure
from skimage.transform import rotate
from skimage.viewer import ImageViewer

from gym.utils import seeding

class GazeboMazeMultiTurtlebotLidarEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboMazeMultiTurtlebotLidar_v0.launch")
        self.t1_vel_pub = rospy.Publisher('/turtle_1/mobile_base/commands/velocity', Twist, queue_size=5)
        self.t2_vel_pub = rospy.Publisher('/turtle_2/mobile_base/commands/velocity', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(4) #F,L,R,B
        self.reward_range = (-np.inf, np.inf)

        self._seed()

        self.img_rows = 32
        self.img_cols = 32
        self.img_channels = 1

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, actionprep):
        action, prep = actionprep
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        active_turtle = self.t2_vel_pub 
        if not prep:
            active_turtle = self.t1_vel_pub

        if action == 0: # FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.4
            vel_cmd.angular.z = 0.0
            active_turtle.publish(vel_cmd)
        elif action == 1: # RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = -0.4
            active_turtle.publish(vel_cmd)
        elif action == 2: # LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = 0.4
            active_turtle.publish(vel_cmd)
        elif action == 3: # BACK
            vel_cmd = Twist()
            vel_cmd.linear.x = -0.4
            vel_cmd.angular.z = 0.0
            active_turtle.publish(vel_cmd)

        # Use seeker vision to determine hider and seeker rewards
        seeker_image = None
        success = False
        cv_image = None
        while seeker_image is None or success is False:
            try:
                seeker_image = rospy.wait_for_message('/turtle_1/camera/rgb/image_raw', Image, timeout=5)
                h = seeker_image.height
                w = seeker_image.width
                cv_image = CvBridge().imgmsg_to_cv2(seeker_image, "bgr8")
                #temporal fix, check image is not corrupted
                if not (cv_image[h//2,w//2,0]==178 and cv_image[h//2,w//2,1]==178 and cv_image[h//2,w//2,2]==178):
                    success = True
                else:
                    pass
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        hider_reward = 0
        seeker_reward = 0

        if not prep:
            # calculate hider seeker rewards from cv_image
            pass

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.resize(cv_image, (self.img_rows, self.img_cols))
        state = cv_image.reshape(1, 1, cv_image.shape[0], cv_image.shape[1])

        return state, (hider_reward, seeker_reward), (seeker_reward >= 10), {}


    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Use seeker vision to determine hider and seeker rewards
        seeker_image = None
        success = False
        cv_image = None
        while seeker_image is None or success is False:
            try:
                seeker_image = rospy.wait_for_message('/turtle_1/camera/rgb/image_raw', Image, timeout=5)
                h = seeker_image.height
                w = seeker_image.width
                cv_image = CvBridge().imgmsg_to_cv2(seeker_image, "bgr8")
                #temporal fix, check image is not corrupted
                if not (cv_image[h//2,w//2,0]==178 and cv_image[h//2,w//2,1]==178 and cv_image[h//2,w//2,2]==178):
                    success = True
                else:
                    pass
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.resize(cv_image, (self.img_rows, self.img_cols))
        state = cv_image.reshape(1, 1, cv_image.shape[0], cv_image.shape[1])

        return state
