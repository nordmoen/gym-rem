#!/usr/bin/env python

"""
Example to test rotation of modules
"""

from gym_rem.envs import ModularEnv
from gym_rem.morph import Rect


env = ModularEnv()
env.render()
robot = Rect()
robot += Rect(3)
robot += Rect(1)
robot += Rect(3)
robot += Rect(0)
env.reset(robot)
while True:
    env.render()
