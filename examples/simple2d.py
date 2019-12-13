#!/usr/bin/env python

"""
Simple 2D simulation
"""
import gym
from gym_rem.morph.two import Rect, Servo


# Create OpenAI Gym Environment
env = gym.make("ModularLocomotion2D-v0")
robot = Rect(1)
srv = Servo(0)
robot += srv
# srv += Rect(0)
# srv += Servo()
# srv += Servo()
# up = Rect()
# robot += up
# up += Rect()
# up += Rect()
# up += Servo()
env.reset(morphology=robot)
while True:
    # env.step(env.action_space.sample())
    env.render()
