#!/usr/bin/env python

"""
OpenAI Gym environment registration
"""
from gym.envs.registration import register


try:
    register(id="ModularLocomotion3D-v0",
             entry_point="gym_rem.envs:Env3D",
             max_episode_steps=240 * 20)
except ImportError:
    pass

try:
    register(id="ModularLocomotion2D-v0",
             entry_point="gym_rem.envs:Env2D")
except ImportError:
    pass
