#!/usr/bin/env python

from .exception import *
from .module import Module
try:
    from .two.abstract import Module2D
except ModuleNotFoundError:
    pass
try:
    from .three.abstract import Module3D
except ModuleNotFoundError:
    # Indicates that PyBullet is not installed
    pass

__all__ = ['Module', 'Module2D', 'Module3D']
