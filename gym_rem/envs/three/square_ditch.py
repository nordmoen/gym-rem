#!/usr/bin/env python

"""
Deceptive 3D Gym environment based on ditches
"""
from gym_rem.envs.abstract_three import ModularEnv, PNGTerrain


class SquareDitchEnv(ModularEnv):
    """3D environment with square ditch like height map terrain"""
    def __init__(self):
        # terrain = PNGTerrain('heightmaps/square_ditch.png',
                             # scale=[.010, .010, 0.10],
                             # position=[0., 0., -0.035])
        terrain = PNGTerrain('heightmaps/square_ditch.png',
                             scale=[.015, .015, 0.18],
                             position=[0., 0., -0.045])
        super().__init__(terrain)


