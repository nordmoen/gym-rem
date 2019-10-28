#!/usr/bin/env python

"""
Movable servo module
"""
from .exception import (NoModuleAttached, ModuleAttached, NoAvailable,
                        ConnectionObstructed)
from .module import Module
from gym_rem.utils import Rot
from enum import Enum
import numpy as np
import pybullet as pyb

# Size of this module
SIZE = np.array([0.103, 0.061])


class Connection(Enum):
    z_plus = (0., 0., 1.)
    z_minus = (0., 0., -1.)
    x_plus = (1., 0., 0.)


class Servo(Module):
    """Movable servo module"""
    def __init__(self, theta=0):
        self.theta = theta % 4
        self.connection_axis = np.array([1., 0., 0.])
        self.orientation = Rot.from_axis(self.connection_axis,
                                         -self.theta * (np.pi / 2.0))
        # NOTE: The fudge factor is to avoid colliding with the plane once
        # spawned
        self.position = np.array([0., 0., SIZE[1] / 2.0 + 0.002])
        self._children = {}
        self.connection_id = 0

    def rotate(self, theta):
        """Update rotation about connection axis"""
        self.theta = (self.theta + theta) % 4
        axis = self.orientation.rotate(self.connection_axis)
        self.orientation += Rot.from_axis(axis, -self.theta * (np.pi / 2.))
        self.update_children()

    @property
    def children(self):
        res = []
        for conn in Connection:
            if conn in self._children:
                res.append(self._children[conn])
        return res

    @property
    def available(self):
        res = []
        for conn in Connection:
            if conn not in self._children:
                res.append(conn)
        return res

    @property
    def joint(self):
        return {'controlMode': pyb.POSITION_CONTROL,
                'jointIndex': 0,
                'maxVelocity': 10.6}

    def __contains__(self, item):
        if isinstance(item, Connection):
            return item in self._children.keys()
        else:
            return super().__contains__(item)

    def __getitem__(self, key):
        if not isinstance(key, Connection):
            raise TypeError("Key: '{}' is not a Connection type".format(key))
        if key not in self._children:
            raise NoModuleAttached("No module attached at: {}".format(key))
        return self._children[key]

    def __delitem__(self, key):
        # Check that 'key' is correct type
        if not (isinstance(key, Connection) or isinstance(key, Module)):
            raise TypeError("Key: '{}' is not a supported type".format(key))
        # If we are asked to delete a module from our self
        if isinstance(key, Module):
            for conn, child in self._children.items():
                if child == key:
                    key = conn
                    break
            else:
                # The module is not a child of this servo
                raise KeyError("Module: {} has no child module: {}"
                               .format(self, key))
        # Check if key is in children
        if key not in self._children:
            raise NoModuleAttached("No module attached at: {}".format(key))
        # First get reference to child so that we can update
        child = self._children[key]
        # Delete child from our children
        del self._children[key]
        # Update child so that its position and orientation is updated
        child.update()

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
        position = self.position + (direction * SIZE[1]) / 2.
        # Update parent pointer of module
        module.update(self, position, direction)

    def update(self, parent=None, pos=None, direction=None):
        # Update own orientation first in case we have been previously
        # connected
        self.orientation = Rot.from_axis(self.connection_axis,
                                         -self.theta * (np.pi / 2.))
        # Update position in case parent is None
        self.position = np.array([0., 0., SIZE[1] / 2.0 + 0.002])
        # Reset connection in case parent is None
        self.connection = None
        # Call super to update orientation
        super().update(parent, pos, direction)
        # If parent is not None we need to update position and connection point
        if self.parent is not None:
            # Update center position for self
            self.position = pos + (direction * SIZE[0]) / 2.
            # Calculate connection points for joint
            conn = np.array([-SIZE[1] / 2., 0., 0.])
            parent_conn = parent.orientation.T.rotate(pos - parent.position)
            self.connection = (parent_conn, conn)
        # Update potential children
        self.update_children()

    def update_children(self):
        for conn in self._children:
            direction = self.orientation.rotate(np.array(conn.value))
            position = self.position + (direction * SIZE[1]) / 2.
            self._children[conn].update(self, position, direction)

    def spawn(self):
        orient = self.orientation.as_quat()
        return pyb.loadURDF('servo/Servo.urdf',
                            basePosition=self.position,
                            baseOrientation=orient)
