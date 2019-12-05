#!/usr/bin/env python

"""
Simple 2D simulation
"""
import gym
from gym_rem.morph.two import Rect, Servo


# Create OpenAI Gym Environment
env = gym.make("ModularLocomotion2D-v0")
robot = Rect(0)
robot += Servo()
up = Servo()
robot += up
up += Rect()
up += Rect()
up += Rect()
env.reset(robot)
while True:
    # env.step(env.action_space.sample())
    env.render()
