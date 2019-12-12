#!/usr/bin/env python

"""
Simple example to show of how to start and simulate a modular robot
"""
from gym_rem.morph.three import Servo, Rect
import gym
import numpy as np


# Create environment
env = gym.make("ModularLocomotion3D-v0")
# Create modular robot to work with
robot = Rect(2)
back = Rect()
robot += back
back += Servo()
side = Rect(2)
back += side
side += Rect()
side += Servo()
side += Servo()
# Call render before creating robot and 'reset' since it will reset the
# environment to enable GUI
env.render()
# Reset environment passing our robot to be spawned
env.reset(morphology=robot)
# Regular old gym loop:
i = 0
num_sec = (1. / env.dt) * 0.25
while True:
    # cmd = np.sin(i / num_sec) * 1.57
    # action = np.ones(len(robot)) * cmd
    # env.step(action)
    env.render()
    i += 1
