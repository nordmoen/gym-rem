#!/usr/bin/env python

"""
3D Gym environment with ripples going outwards
"""
from gym_rem.envs.abstract_three import ModularEnv, PNGTerrain


class RippleEnv(ModularEnv):
    """3D environment with Colosseum like height map terrain"""
    def __init__(self):
        terrain = PNGTerrain('heightmaps/custom3.png',
                             scale=[.015, .015, 0.1])
        super().__init__(terrain)
