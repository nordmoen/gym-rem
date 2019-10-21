#!/usr/bin/env python

"""
Simple example to show of how to start and simulate a modular robot
"""

from gym_rem.morph.servo import Servo, Connection
import gym
import logging
import numpy as np

# Setup logging if desired:
logging.basicConfig(level=logging.INFO)
# Create environment
env = gym.make("ModularLocomotion-v0")
# Call render before creating robot and 'reset' since it will reset the
# environment to enable GUI
env.render()
# Create modular robot to work with
robot = Servo()
curr = robot
for _ in range(10):
    curr[Connection.x_plus] = Servo()
    curr = curr[Connection.x_plus]
up = Servo()
# robot += up
# Reset environment passing our robot to be spawned
env.reset(morphology=robot)
# Regular old gym loop:
i = 0
num_sec = 240. * 10
while True:
    cmd = np.sin(i / num_sec) * 1.57
    action = np.ones(len(robot)) * 1.57
    env.step(action)
    env.render()
    i += 1
