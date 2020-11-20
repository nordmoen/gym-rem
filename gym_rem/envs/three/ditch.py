#!/usr/bin/env python

"""
Deceptive 3D Gym environment based on ditches
"""
from gym_rem.envs.abstract_three import ModularEnv, PNGTerrain


class DitchEnv(ModularEnv):
    """3D environment with ditch like height map terrain"""
    def __init__(self):
        terrain = PNGTerrain('heightmaps/ditch2.png',
                             scale=[.015, .015, 0.1],
                             position=[0., 0., -0.035])
        super().__init__(terrain)

