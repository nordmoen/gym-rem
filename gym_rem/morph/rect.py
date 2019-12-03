#!/usr/bin/env python

"""
Non-movable Rectangle module
"""
from .exception import ModuleAttached, ConnectionObstructed
from .module import Module
from enum import Enum
from gym_rem.utils import Rot
import numpy as np
import pybullet as pyb


class Connection(Enum):
    """Available connections for Rect module"""
    x_plus = (1., 0., 0.)
    y_plus = (0., 1., 0.)
    x_minus = (-1., 0., 0.)
    y_minus = (0., -1., 0.)
    z_plus = (0., 0., 1.)


class Rect(Module):
    """Non-movable rectangle module"""

    def __init__(self, theta=0, size=(0.061, 0.061, 0.061)):
        self.theta = theta % 4
        self.size = np.array(size)
        assert self.size.shape == (3,), "Size must be a 3 element vector!"
        self.connection_axis = np.array([0., 0., 1.])
        self.orientation = Rot.from_axis(self.connection_axis,
                                         -self.theta * (np.pi / 2.))
        # NOTE: The fudge factor is to avoid colliding with the plane once
        # spawned
        self.position = np.array([0., 0., self.size[2] / 2. + 0.002])
        self.connection_type = Connection
        self._children = {}

    def rotate(self, theta):
        """Update rotation about connection axis"""
        self.theta = (self.theta + theta) % 4
        axis = self.connection_axis
        self.orientation += Rot.from_axis(axis, -self.theta * (np.pi / 2.))
        self.update_children()

    def __setitem__(self, key, module):
        if not isinstance(key, Connection):
            raise TypeError("Key: '{}' is not a Connection type".format(key))
        if key in self._children:
            raise ModuleAttached()
        if key not in self.available:
            raise ConnectionObstructed()
        # Add module as a child
        self._children[key] = module
        # Calculate connection point
        direction = self.orientation.rotate(np.array(key.value))
        position = self.position + (direction * self.size) / 2.
        # Update parent pointer of module
        module.update(self, position, direction)

    def update(self, parent=None, pos=None, direction=None):
        # Update own orientation first in case we have been previously
        # connected
        self.orientation = Rot.from_axis(self.connection_axis,
                                         -self.theta * (np.pi / 2.))
        # Update position in case parent is None
        self.position = np.array([0., 0., self.size[2] / 2. + 0.002])
        # Reset connection in case parent is None
        self.connection = None
        # Call super to update orientation
        super().update(parent, pos, direction)
        # If parent is not None we need to update position and connection point
        if self.parent is not None:
            # Update center position for self
            # NOTE: We add a little fudge factor to avoid overlap
            self.position = pos + (direction * self.size * 1.01) / 2.
            # Calculate connection points for joint
            conn = np.array([0., 0., -self.size[2] / 2.])
            parent_conn = parent.orientation.T.rotate(pos - parent.position)
            self.connection = (parent_conn, conn)
        # Update potential children
        self.update_children()

    def update_children(self):
        for conn in self._children:
            direction = self.orientation.rotate(np.array(conn.value))
            position = self.position + (direction * self.size) / 2.
            self._children[conn].update(self, position, direction)

    def spawn(self, client):
        orient = self.orientation.as_quat()
        return client.loadURDF('rect.urdf',
                               basePosition=self.position,
                               baseOrientation=orient)
