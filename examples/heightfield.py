#!/usr/bin/env python

"""
Simple example to show of how to use a different terrain based on heightfields
"""
from gym_rem.morph.three import Servo, Rect
from gym_rem.envs import PNGTerrain
import gym


# First create terrain
terrain = PNGTerrain('heightmaps/custom1.png',
                     scale=[.02, .02, 0.1])
# Create environment
env = gym.make("ModularLocomotion3D-v0", terrain=terrain)
# Alternatively:
env = gym.make("ModularCollosseum3D-v0")
# Create modular robot to work with
robot = Rect()
robot += Servo(1)
robot += Servo()
robot += Rect()
# Call render before creating robot and 'reset' since it will reset the
# environment to enable GUI
env.render()
# Reset environment passing our robot to be spawned
env.reset(morphology=robot)
# Regular old gym loop:
while True:
    env.step([0., 0.])
    env.render()
