#!/usr/bin/env python
try:
    from .abstract_three import ModularEnv as Env3D
except ModuleNotFoundError:
    # This indicates that PyBullet is not installed
    pass
try:
    from .abstract_two import ModularEnv2D as Env2D
except ModuleNotFoundError as e:
    # Box2D not installed
    pass

__all__ = ['Env2D', 'Env3D']
