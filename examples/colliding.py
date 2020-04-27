#!/usr/bin/env python

"""
Simple example to show of the return value when modules are colliding
"""

from gym_rem.morph.three.servo import Servo, Connection
import gym
import logging
import time

# Setup logging if desired:
logging.basicConfig(level=logging.INFO)
# Create environment
env = gym.make("ModularLocomotion3D-v0")
# Call render before creating robot and 'reset' since it will reset the
# environment to enable GUI
env.render()
# Create modular robot to work with
robot = Servo()
robot += Servo()
robot += Servo()
robot[Connection.x_plus] = Servo()
up = robot[Connection.z_plus]
up[Connection.z_minus] = Servo()
side = up[Connection.z_minus]
# NOTE: This will be spawned, but quickly removed once overlap is detected
overlapper = Servo()
side[Connection.z_minus] = overlapper
# Reset environment passing our robot to be spawned
env.reset(morphology=robot)
# NOTE: We can inspect the spawned robot through 'env.morphology'
# print(env.morphology)
# Regular old gym loop:
while True:
    env.render()
    time.sleep(0.5)
