#!/usr/bin/env python
try:
    from .abstract_three import ModularEnv as Env3D
    from .three.colosseum import ColosseumEnv
    from .abstract_three import PNGTerrain, ArrayTerrain
except ModuleNotFoundError:
    # This indicates that PyBullet is not installed
    pass
try:
    from .abstract_two import ModularEnv2D as Env2D
except ModuleNotFoundError:
    # Box2D not installed
    pass

__all__ = ['Env2D', 'Env3D', 'PNGTerrain', 'ArrayTerrain', 'ColosseumEnv']
