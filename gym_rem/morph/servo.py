#!/usr/bin/env python

"""
Movable servo module
"""
from .exception import (NoModuleAttached, ModuleAttached, NoAvailable,
                        ConnectionObstructed)
from .module import Module
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
        self.orientation = np.array([-(theta % 4) * (np.pi / 2.0), 0., 0.])
        self.position = np.array([0., 0., 0.0305])
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
                'maxVelocity': 0.1}

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
        direction = self._mat.dot(np.array(key.value))
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

    @property
    def _mat(self):
        """Helper method to get orientation as rotation matrix"""
        quat = pyb.getQuaternionFromEuler(self.orientation)
        mat = np.reshape(pyb.getMatrixFromQuaternion(quat), (3, 3))
        return mat

    def update(self, parent, pos, direction):
        if self.parent is not None:
            raise ModuleAttached("This module already has a parent: {}"
                                 .format(self.parent))
        self.parent = parent
        self.position = pos + (direction * SIZE[0] * 1.1) / 2.
        angle = np.arccos(direction.dot(np.array([1., 0, 0])))
        self.orientation = parent.orientation + Servo.to_euler(direction, angle)
        fudge = SIZE[1] / 2.
        self.connection = (direction * fudge,
                           (-fudge, 0., 0.),
                           direction)
        print(self.connection)

    def __repr__(self):
        return "Servo(pos: {}, orient: {})".format(self.position,
                                                   self.orientation)

    @staticmethod
    def to_euler(axis, angle):
        """Convert axis angle representation to euler"""
        x, y, z = axis
        s = np.sin(angle)
        c = np.cos(angle)
        t = 1. - c
        heading = np.arctan2(y * s - x * z, 1. - (y ** 2 + z**2) * t)
        attitude = np.arcsin(x * y * t + z * s)
        bank = np.arctan2(x * s - y * z * t, 1. - (x**2 + z**2) * t)
        # return bank, -attitude, heading
        return heading, -attitude, bank

    def spawn(self):
        orient = pyb.getQuaternionFromEuler(self.orientation)
        return pyb.loadURDF('servo/Servo.urdf',
                            basePosition=self.position,
                            baseOrientation=orient)
