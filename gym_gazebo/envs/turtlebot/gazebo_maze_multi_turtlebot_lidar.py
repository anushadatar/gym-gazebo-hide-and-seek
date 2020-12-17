"""
Script associated with robot operation, computes and returns robot state for each step.
"""
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

class GazeboMazeMultiTurtlebotLidarEnv(gazebo_env.GazeboEnv):
    """
    Environment associated with the multiple turtlebot hide and seek simulation.
    """
    def __init__(self):
      	"""
	Initialize the simulation, image processing variables, and rewards.
	"""
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
        self.last50actions = [0]*50

        self.img_rows = 32
        self.img_cols = 32
        self.img_channels = 1
	
	self.hider_reward_increment = 1
	self.seeker_reward_increment = 2


    def _seed(self, seed=None):
	"""
	Get the random seed for beginning environment operation.
	
	seed : Input seed value, defaults to None.
	Returns seed value.
	"""
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def in_view(self, gray_cv_image):
	"""
	Returns whether or not the other robot is in view.
	gray_cv_image: The image associated with the robot's camera view.

	Returns true if robot in view, false otherwise.
	"""
        turtlebot_detected = False
        
	#cv2.imshow("in_view test", gray_cv_image)
        
	#ret, thresh = cv2.threshold(gray_cv_image, 25, 255, cv2.THRESH_BINARY_INV)
        #contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #if len(contours) > 0:
        #    turtlebot_detected = True
        return turtlebot_detected

    def step(self, actionprep):
	"""
	Update the reward values for each step.
	"""
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
        
        self.last50actions.pop(0)
        if action in [0,3]:
            self.last50actions.append(0)
        else:
            self.last50actions.append(1)
        action_sum = sum(self.last50actions)

        # Use the seeker's vision to determine hider and seeker rewards
        seeker_image = None
        success = False
        cv_image = None
        while seeker_image is None or success is False:
            try:
                seeker_image = rospy.wait_for_message('/turtle_1/camera/rgb/image_raw', Image, timeout=5)
                h = seeker_image.height
                w = seeker_image.width
                cv_image = CvBridge().imgmsg_to_cv2(seeker_image, "bgr8")
                # Temporal fix to check image is not corrupted.
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
	in_view = False
        if cv_image.min() < 60:
	    in_view = True
	#hsv_image = cv2.cvtColor(cv_image. cv2.COLOR_BGR2HSV)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.resize(cv_image, (self.img_rows, self.img_cols))
	

        hider_reward = 0
        seeker_reward = 0
        done = False

        if not prep:
            # Calculate hider seeker rewards from cv_image
            if in_view:
                #print("IN VIEW")
                done = True
                seeker_reward += self.seeker_reward_increment
                hider_reward -= self.hider_reward_increment
            else:
                seeker_reward -= self.seeker_reward_increment
                hider_reward += self.hider_reward_increment
            if action_sum > 40:
                seeker_reward += 0.2*self.seeker_reward_increment

        state = cv_image.reshape(1, 1, cv_image.shape[0], cv_image.shape[1])

        return state, hider_reward - seeker_reward, done, {}


    def reset(self):
        """
	Reset the environment state.
	"""
        self.last50actions = [0]*50
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
