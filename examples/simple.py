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
robot = Servo(2)
robot += Servo()
up = robot[Connection.z_plus]
# up[Connection.z_plus] = Servo()
# up += Servo()
# up += Servo()
# up += Servo()
# Reset environment passing our robot to be spawned
env.reset(morphology=robot)
# Regular old gym loop:
i = 0
num_sec = 240. * 4.
while True:
    cmd = np.sin(i / num_sec) * 1.57
    action = np.ones(len(robot)) * cmd
    # env.step(action)
    env.render()
    i += 1
