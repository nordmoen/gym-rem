#!/usr/bin/env python

"""
Simple 2D simulation
"""
import gym
from gym_rem.morph.two import Rect, Servo


# Create OpenAI Gym Environment
env = gym.make("ModularLocomotion2D-v0")
robot = Rect()
robot += Servo()
env.reset(morphology=robot)
while True:
    env.step([3.14 / 4.])
    env.render()
