#!/usr/bin/env python

"""
Abstract module which all morphological modules must extend
"""
from .exception import ModuleAttached, NoAvailable
from collections import deque
from gym_rem.utils import Rot
import numpy as np


class Module(object):
    """Abstract morphological module"""
    # Possible connected parent module
    parent = None
    # Connection to parent, should be 'None' or a tuple of position, direction
    connection = None
    # Center position of module in global space
    position = np.zeros(3)
    # Orientation of module
    orientation = Rot.identity()
    # Axis to rotate module around when attaching
    connection_axis = np.array([1., 0., 0.])
    # Connection ID to use when creating constraint
    connection_id = -1

    @property
    def children(self):
        """
        Get a list of the immediate children of this module.

        This is different from '__iter__' in that this method only returns the
        children directly connected to this module and not their children.
        """
        return []

    @property
    def available(self):
        """
        Get the list of available connection points that can be used to add new
        modules.

        This function should return a list of items corresponding to this
        modules access type.
        """
        return []

    @property
    def joint(self):
        """
        Get the joint configuration as a dict.

        The joint configuration is a leaky abstraction used for movable joints.
        Subclasses should return a dict with joint configuration according to
        pybyllet, supported items are 'controlMode' (required), 'jointIndex'
        (required), 'positionGain' (optional), 'velocityGain' (optional) and
        'maxVelocity' (optional).

        Non-movable joints should return None.
        """
        return None

    @property
    def root(self):
        """Extract the root module of this module.

        This method will iterate through all parents until the root is found.
        If this module does not have a parent `self` will be returned."""
        queue = self
        while True:
            if queue.parent is None:
                return queue
            else:
                queue = queue.parent

    def connection_point(self, item):
        """Get the connection point associated with 'item'"""
        if not isinstance(item, Module):
            raise TypeError("Cannot connect {} to modules".format(item))
        if item not in self:
            raise KeyError("This module has no child: '{}'".format(item))
        raise NotImplementedError("Not supported")

    def __iter__(self):
        """Iterate all modules connected to this module"""
        yield self
        queue = deque([self])
        while len(queue) > 0:
            for child in queue.popleft().children:
                yield child
                queue.append(child)

    def __contains__(self, item):
        """Check if the module 'item' is connected to this module"""
        if not isinstance(item, Module):
            raise TypeError("Cannot connect {} to modules".format(item))
        return item in self.children

    def __len__(self):
        """
        Get the size of the morphology represented by this module.

        This method should count the length of all children also connected to
        this module.
        """
        return 1 + sum([len(m) for m in self.children])

    def __getitem__(self, key):
        """Return the child connected at 'key'"""
        raise NotImplementedError("Not supported")

    def __delitem__(self, key):
        """Remove the connection to child connected at 'key'"""
        raise NotImplementedError("Not supported")

    def __setitem__(self, key, module):
        """Attached the child 'module' at the point 'key'"""
        raise NotImplementedError("Not supported")

    def __iadd__(self, module):
        """Attach the child 'module' at the next available attachment point"""
        for conn in self.available:
            self[conn] = module
            return self
        raise NoAvailable("There were no available or non-obstructed free\
                spaces")

    def __repr__(self):
        """Create print friendly representation of this module"""
        return "{}({})".format(self.__class__.__name__, self.position)

    def update(self, parent=None, pos=None, direction=None):
        """Update configuration for the module.

        This function should update the parent pointer and internal position of
        the module. The 'pos' argument is the central point where this module
        connects to parent. The 'direction' argument is a vector pointing in
        the direction the module should be attached at.

        If no arguments are given it indicates an update back to center
        position."""
        if parent is not None and self.parent is not None and parent != self.parent:
            raise ModuleAttached("This module already has a parent: {}"
                                 .format(self.parent))
        self.parent = parent
        # NOTE: We accept 'None' parents which could indicate a detachment of
        # this module
        if parent is not None:
            # The below equation rotates the 'connection_axis' parameter to
            # look in the same direction as 'direction'
            v = np.cross(self.connection_axis, direction)
            c = self.connection_axis.dot(direction)
            skew = np.array([[0., -v[2], v[1]],
                             [v[2], 0., -v[0]],
                             [-v[1], v[0], 0.]])
            if 1. + c != 0.:
                orient = Rot(np.identity(3) + skew
                             + skew.dot(skew) * (1. / (1. + c)))
            else:
                # This means that 'connection_axis' and 'direction' point in
                # the same direction, to convert we can simply flip one axis
                # and rotate by pi
                orient = Rot.from_axis(np.flip(self.connection_axis), np.pi)
            # Update own orientation
            # self.orientation += parent.orientation + orient
            self.orientation += orient

    def update_children(self):
        """Update all child modules of self"""
        raise NotImplementedError("Not supported")

    def spawn(self):
        """Spawn the module in the physics simulation"""
        raise NotImplementedError("Not supported")
