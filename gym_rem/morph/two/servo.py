#!/usr/bin/env python

"""
Simple 2D servo
"""
from .abstract import Module2D
from enum import Enum
from gym_rem.morph import ModuleAttached
from gym_rem.utils import Rot
import numpy as np


class Connection(Enum):
    """Available connection for 2D servo module"""
    z_minus = (0., 0., -1.)
    x_plus = (1., 0., 0.)
    z_plus = (0., 0., 1.)


class Servo(Module2D):
    """2D servo with revolute joint"""
    def __init__(self, theta=0):
        self.theta = theta % 2
        self.connection_axis = np.array([-1., 0., 0.])
        self.orientation = Rot.from_axis(self.connection_axis,
                                         -self.theta * np.pi)
        self.position = np.array([0., 0., 20.])
        self.connection_type = Connection
        self._children = {}

    def rotate(self, theta):
        """Update internal rotation about connection axis"""
        self.theta = (self.theta + theta) % 2
        self.orientation += Rot.from_axis(self.connection_axis,
                                          -self.theta * np.pi)
        self.update_children()

    def __setitem__(self, key, module):
        if not isinstance(key, Connection):
            raise TypeError("Key: '{}' is not a Connection type".format(key))
        if key in self._children:
            raise ModuleAttached()
        # Add child to children
        self._children[key] = module
        # Calculate direction of child connection
        direction = self.orientation.T.rotate(np.array(key.value))
        # Calculate position on self of child
        position = self.position + (direction * 20.)
        # Update child
        module.update(self, position, direction)

    def update(self, parent=None, pos=None, direction=None):
        # Update own orientation first in case we have been previously
        # connected
        self.orientation = Rot.from_axis(self.connection_axis,
                                         -self.theta * np.pi)
        # Update position in case parent is None
        self.position = np.array([0., 0., 20.])
        # Reset connection in case parent is None
        self.connection = None
        # Call super to update orientation
        super().update(parent, pos, direction)
        # If parent is not None we need to update position and connection point
        if self.parent is not None:
            # Update center position for self
            # NOTE: We add a little fudge factor to avoid overlap
            self.position = pos + (direction * (20.1 + 14.5))
            # Calculate connection points for joint
            conn = np.array([-4.1, 0., 0.])
            parent_conn = parent.orientation.rotate(pos - parent.position)
            self.connection = (parent_conn, conn)
        # Update potential children
        self.update_children()

    def update_children(self):
        for conn in self._children:
            direction = self.orientation.T.rotate(np.array(conn.value))
            position = self.position + (direction * (20.1 + 14.5))
            self._children[conn].update(self, position, direction)
