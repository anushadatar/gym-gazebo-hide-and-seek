#!/usr/bin/env python
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
    render_skip = 0 #Skip first X episodes.
    render_interval = 50 #Show render Every Y episodes.
    render_episodes = 10 #Show Z episodes every rendering.

    if (x%render_interval == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x-render_episodes)%render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)

if __name__ == '__main__':

    env = gym.make('GazeboMazeMultiTurtlebotLidar-v0')

    outdir = '/tmp/gazebo_gym_experiments'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    plotter = liveplot.LivePlot(outdir)

    last_time_steps = numpy.ndarray(0)

    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=0.1, gamma=0.9, epsilon=0.9)

    initial_epsilon = qlearn.epsilon

    epsilon_discount = 0.998 # 1098 eps to reach 0.1

    start_time = time.time()
    total_episodes = 10000
    highest_reward = 0

    for x in range(total_episodes):
        prep_done = False
        done = False

        seeker_cumulated_reward = 0
        hider_cumulated_reward = 0

        observation = env.reset()

        if qlearn.epsilon > 0.1:
            qlearn.epsilon *= epsilon_discount

        state = ''.join(map(str, observation))

        # PREP PHASE - HIDERS HIDE
        for i in range(500):
            # Pick an action based on the current state
            action = qlearn.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step((action,True))
            
            # No reward during prep phase
            nextState = ''.join(map(str, observation))
            qlearn.learn(state, action, 0, nextState)
            env._flush(force=True)

            # No early exiting prep
            state = nextState

        # SEARCH PHASE - SEEKERS SEEK
        for i in range(500):
            # Pick an action based on the current state
            action = qlearn.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step((action,False))
            hider_reward, seeker_reward = reward
            hider_cumulated_reward += hider_reward
            seeker_cumulated_reward += seeker_reward

            if highest_reward < (hider_cumulated_reward - seeker_cumulated_reward):
                highest_reward = hider_cumulated_reward - seeker_cumulated_reward

            nextState = ''.join(map(str, observation))

            qlearn.learn(state, action, hider_cumulated_reward - seeker_cumulated_reward, nextState)

            env._flush(force=True)

            # Done when seeker hits saturated reward
            if not(seeker_cumulated_reward > 4):
                state = nextState
            else:
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break
        
        done = True
        if x%100==0:
            plotter.plot(env)

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(hider_cumulated_reward-seeker_cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))

    #Github table content
    print ("\n|"+str(total_episodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |")

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()
