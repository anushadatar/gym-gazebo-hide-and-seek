"""
Script associated with robot operation, computes and returns robot state for each step.
"""
import numpy as np
import os
import random
import sys
import time

# Gym dependencies.
import gym
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from gym.utils import seeding

# Ros Dependencies
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import rospy
import roslaunch

# CV Dependencies
import cv2
from cv_bridge import CvBridge, CvBridgeError
import skimage as skimage
from skimage import transform, color, exposure
from skimage.transform import rotate
from skimage.viewer import ImageViewer

class GazeboMazeMultiTurtlebotLidarEnv(gazebo_env.GazeboEnv):
    """
    Environment associated with the multiple turtlebot hide and seek simulation.
    Inherits from existing gazebo environment, so serves as the "environment" portion
    required for a functioning OpenAI gym instance.
    """
    def __init__(self):
      	"""
	Initialize the simulation, image processing variables, and reward parameters.
	"""
	# Launch the multiple turtlebot simulation.
        gazebo_env.GazeboEnv.__init__(self, "GazeboMazeMultiTurtlebotLidar_v0.launch")
	# Publisher associated with the first turtlebot (the seeker).
        self.seeker_publisher = rospy.Publisher('/turtle_1/mobile_base/commands/velocity', Twist, queue_size=5)
	# Publisher associated with the second turtlebot (the hider). 
        self.hider_publisher = rospy.Publisher('/turtle_2/mobile_base/commands/velocity', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

	# Get the random seed for beginning environment operation.
        self._seed()
	# Store the last 50 actions to intervene in the case of infinite looping.
        self.last50actions = [0]*50
	self.action_space = spaces.Discrete(4)
	
	# Network image processing parameters.
        self.img_rows = 32
        self.img_cols = 32
        self.img_channels = 1
	
	# Value by which to increment or decrement the seeker reward based on if the robot is visible/
	self.reward_delta = 1

    def _seed(self, seed=None):
	"""
	Get the random seed for beginning environment operation.
	
	seed : Input seed value, defaults to None.
	Returns seed value.
	"""
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, actionprep):
	"""
	Update the reward values for each step.
	actionprep = The state that the overall program is currently in (where
		     "prep" indicates that the hider is hiding)
	
	Returns 
	state      = 
 	reward     = 
	done       = {}
	"""
        action, prep = actionprep
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        active_turtle = self.hider_publisher 
        if not prep:
            active_turtle = self.seeker_publisher

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

        # Use the robot's vision to determine associated reward.
	image = None
        success = False
        cv_image = None
        while image is None or success is False:
            try:
                if prep: # Get hider's camera if in prep mode.
		     image = rospy.wait_for_message('/turtle_2/camera/rgb/image_raw', Image, timeout=5)
                else: # Otherwise, simply use seeker's camera.
		     image = rospy.wait_for_message('/turtle_1/camera/rgb/image_raw', Image, timeout=5)
		h = image.height
                w = image.width
                cv_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
                # Check image is not corrupted.
                if not (cv_image[h//2,w//2,0]==178 and cv_image[h//2,w//2,1]==178 and cv_image[h//2,w//2,2]==178):
                    success = True
                else:
                    pass
            except:
                pass

	# Pause physics.
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
	
	# Check to see if the other robot is in frame.
	# Currently only used in seeking mode, but keeping both is useful for debugging.
	in_view = False
	# Use a simple color threshold to check if the other robot is in frame.
        if min(cv_image.flatten()) < 30:
	    in_view = True
	# Conver cv_image to colors and size needed for state update.
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.resize(cv_image, (self.img_rows, self.img_cols))
	
	# Set reward and done values (used in the context of seeeking),
        reward = 0
        done = False
        if not prep:
            # Calculate hider seeker rewards from cv_image
            if in_view:
                done = True
                reward += self.reward_delta
            else:
                reward -= self.reward_delta
	    # Smooth out the reward delta as needed.
            if action_sum > 40:
                reward += 0.2*self.reward_delta
	# Reshape image and complete the state update.
        state = cv_image.reshape(1, 1, cv_image.shape[0], cv_image.shape[1])
        return state, reward, done, {}

    def reset(self):
        """
	Reset the environment state.
	Returns the state value as the associated CV image.
	"""
	# Zero out the last fifty actions tracker.
        self.last50actions = [0]*50
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
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

        # Reset seeker vision as used to determine hider and seeker rewards.
        seeker_image = None
        success = False
        cv_image = None
        while seeker_image is None or success is False:
            try:
                seeker_image = rospy.wait_for_message('/turtle_1/camera/rgb/image_raw', Image, timeout=5)
                h = seeker_image.height
                w = seeker_image.width
                cv_image = CvBridge().imgmsg_to_cv2(seeker_image, "bgr8")
                # Temporary fix to check that image is not corrupted.
                if not (cv_image[h//2,w//2,0]==178 and cv_image[h//2,w//2,1]==178 and cv_image[h//2,w//2,2]==178):
                    success = True
                else:
                    pass
            except:
                pass

	# Pause physics for final state update.
	rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.resize(cv_image, (self.img_rows, self.img_cols))
        state = cv_image.reshape(1, 1, cv_image.shape[0], cv_image.shape[1])
        return state
