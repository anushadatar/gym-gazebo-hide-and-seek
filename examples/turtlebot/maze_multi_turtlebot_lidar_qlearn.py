#!/usr/bin/env python
"""
Script associated with orchestrating the RL framework with the multi robot 
environment.
"""
import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time
import liveplot
import qlearn

def render():
    """ 
    Set the rendering parameters associated with the environment.
    """	
    render_skip = 0 #Skip first X episodes.
    render_interval = 50 #Show render Every Y episodes.
    render_episodes = 10 #Show Z episodes every rendering.

    if (x%render_interval == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x-render_episodes)%render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)

if __name__ == '__main__':
    """
    Main script associated with initializing the environment, networks, and
    associated parameter values.
    """
    # Use the multi turtlebot environment, which inherits from the general gazebo envrionment.
    env = gym.make('GazeboMazeMultiTurtlebotLidar-v0')

    # Place output data in a temporary directory, and plot output values accordingly.
    outdir = '/tmp/gazebo_gym_experiments'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    plotter = liveplot.LivePlot(outdir)
    # RL network parameter values.
    last_time_steps = numpy.ndarray(0)
    alpha = 0.1
    epsilon = 0.9 
    gamma = 0.9
    epsilon_discount = 0.998 # 1098 eps to reach 0.1
    total_episodes = 10000
    highest_reward = 0

    # Networks for both the hider and the seeker, trained separately but
    # (at least currently) have the same seed parameters.
    hider_qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=alpha, gamma=gamma, epsilon=epsilon)
    seeker_qlearn = qlearn.QLearn(actions=range(env.action_space.n), 
                    alpha=alpha, gamma=gamma, epsilon=epsilon)
  
    # Set the initial time value. 
    start_time = time.time()

    # Orchestrate each individual episode.
    for x in range(total_episodes):
        prep_done = False
        done = False
        cumulated_reward = 0
        observation = env.reset()

	# Scale down epsilon values as needed.
        if hider_qlearn.epsilon > 0.1:
            hider_qlearn.epsilon *= epsilon_discount
	if seeker_qlearn.epsilon > 0.1:
            seeker_qlearn.epsilon *= epsilon_discount
        state = ''.join(map(str, observation))

        for i in range(1000):
	    # Alternate between the prep and seeking phases.	
            if i % 2 == 0:    
		# PREP PHASE - HIDER HIDES.
	    	# Pick an action based on the current state.
            	action = hider_qlearn.chooseAction(state)
            	# Execute the action and get feedback.
            	observation, reward, done, info = env.step((action,True))
            	# No reward during prep phase.
            	nextState = ''.join(map(str, observation))
            	hider_qlearn.learn(state, action, 0, nextState)
            	env._flush(force=True)
            	# No early exiting prep - move on to the next state regardless.
            	state = nextState
   	    else:
		# SEARCH PHASE - SEEKER SEEKS.
            	# Pick an action based on the current state.
            	action = seeker_qlearn.chooseAction(state)
            	# Execute the action and get feedback.
            	observation, reward, done, info = env.step((action,False))
		# Keep track of associated reward functions.
	    	cumulated_reward += reward
            	if highest_reward < cumulated_reward:
                    highest_reward = cumulated_reward
		# Keep track of observations and the associated following state.
                nextState = ''.join(map(str, observation))
		# Continue learning.
                seeker_qlearn.learn(state, action, reward, nextState)
                env._flush(force=True)
                # Done when seeker hits saturated reward value.
                if not done:
                    state = nextState
                else:
                    last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                    break
	
	# Reset the stats recorded as needed following the episode.
        env.stats_recorder.save_complete()
        env.stats_recorder.done = True
        
	# Plot values incrementally and print updates about the output of the seeker network.
        if x%100==0:
            plotter.plot(env)
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("Seeker EP: "+str(x+1)+" - [alpha: "+str(round(seeker_qlearn.alpha,2))+" - gamma: "+str(round(seeker_qlearn.gamma,2))+" - epsilon: "+str(round(seeker_qlearn.epsilon,2))+"] - Reward: "+str(reward)+"     Time: %d:%02d:%02d" % (h, m, s))

    #Github table content and final informational outputs.
    print ("\n|"+str(total_episodes)+"|"+str(seeker_qlearn.alpha)+"|"+str(seeker_qlearn.gamma)+"|"+str(epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |")
    l = last_time_steps.tolist()
    l.sort()
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))
    env.close(seeker_)
