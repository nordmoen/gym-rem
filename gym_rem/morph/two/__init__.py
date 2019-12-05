#!/usr/bin/env python

"""
2D module for modular robotics gym environments
"""

from .abstract import Module2D
from .rect import Rect
from .servo import Servo

__all__ = ['Rect', 'Servo']
