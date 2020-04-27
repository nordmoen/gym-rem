#!/usr/bin/env python

"""
3D Gym environment with a Colosseum like heightmap
"""
from gym_rem.envs.abstract_three import ModularEnv, PNGTerrain


class ColosseumEnv(ModularEnv):
    """3D environment with Colosseum like height map terrain"""
    def __init__(self):
        terrain = PNGTerrain('heightmaps/custom1.png',
                             scale=[.02, .02, 0.1])
        super().__init__(terrain)
