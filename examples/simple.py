#!/usr/bin/env python

"""
Simple example to show of how to start and simulate a modular robot
"""
from gym_rem.morph.three import Servo, Rect
import gym


# Create environment
env = gym.make("ModularLocomotion3D-v0")
# Create modular robot to work with
robot = Rect()
robot += Servo()
servo = Servo()
robot += servo
servo += Rect()
# Call render before creating robot and 'reset' since it will reset the
# environment to enable GUI
env.render()
# Reset environment passing our robot to be spawned
env.reset(morphology=robot)
# Regular old gym loop:
while True:
    # env.step(env.action_space.sample())
    env.step([0., 0.])
    env.render()
