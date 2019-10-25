#!/usr/bin/env python

"""
Simple example to easy play with morphologies interactively
"""
from gym_rem.morph import Servo, Rect
import gym
import time


# Create environment
env = gym.make("ModularLocomotion-v0")
# Call render before creating robot and 'reset' since it will reset the
# environment to enable GUI
env.render()
# Create modular robot to work with
robot = Rect()
robot += Servo()
robot += Servo()
robot += Servo()
robot += Servo()
up = Servo()
robot += up
up += Rect()
up += Rect()
up += Rect()
# robot += Rect()
# robot += Rect()
# robot += Rect()
# Reset environment passing our robot to be spawned
env.reset(morphology=robot)
while True:
    env.render()
    time.sleep(0.1)
