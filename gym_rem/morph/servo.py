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
        self.orientation = Rot.from_euler(-(theta % 4) * (np.pi / 2.0), 0., 0.)
        self.position = np.array([0., 0., SIZE[1] / 2.0])
        self._children = {}

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

    def __getitem__(self, key):
        if not isinstance(key, Connection):
            raise TypeError("Key: '{}' is not a Connection type".format(key))
        if key not in self._children:
            raise NoModuleAttached("No module attached at: {}".format(key))
        return self._children[key]

    def __delitem__(self, key):
        if not isinstance(key, Connection):
            raise TypeError("Key: '{}' is not a Connection type".format(key))
        if key not in self._children:
            raise NoModuleAttached("No module attached at: {}".format(key))
        del self._children[key]

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

    def __iadd__(self, module):
        for conn in Connection:
            if conn not in self._children:
                try:
                    self[conn] = module
                    return self
                except ConnectionObstructed:
                    # Ignore since there could be other available points
                    pass
        raise NoAvailable("There were no available or non-obstructed free\
                spaces")

    def update(self, parent, pos, direction):
        super().update(parent, pos, direction)
        # Update center position for self
        self.position = pos + (direction * SIZE[0]) / 2.
        # Calculate connection points for joint
        conn = np.array([-SIZE[1] / 2., 0., 0.])
        parent_conn = parent.orientation.T.rotate(direction * (SIZE[1] / 2.))
        self.connection = (parent_conn, conn)

    def spawn(self):
        orient = self.orientation.as_quat()
        return pyb.loadURDF('servo/Servo.urdf',
                            basePosition=self.position,
                            baseOrientation=orient)
