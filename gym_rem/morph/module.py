#!/usr/bin/env python

"""
Abstract module which all morphological modules must extend
"""
import numpy as np


class Module(object):
    """Abstract morphological module"""
    # Possible connected parent module
    parent = None
    # Connection to parent, should be 'None' or a tuple of position, direction
    connection = None
    # Center position of module in global space
    position = np.zeros(3)
    # Orientation of module, roll, pitch, yaw
    orientation = np.zeros(3)

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

    def __iter__(self):
        """Iterate all modules connected to this module"""
        yield self
        queue = [self]
        while len(queue) > 0:
            for child in queue.pop().children:
                yield child
                queue.append(child)

    def __len__(self):
        """
        Get the size of the morphology represented by this module.

        This method should count the length of all children also connected to
        this module.
        """
        return 1 + sum([len(m) for m in self.children])

    def __getitem__(self, key):
        """Return the child connected at 'key'"""
        raise NotImplementedError("Abstract class")

    def __delitem__(self, key):
        """Remove the connection to child connected at 'key'"""
        raise NotImplementedError("Abstract class")

    def __setitem__(self, key, module):
        """Attached the child 'module' at the point 'key'"""
        raise NotImplementedError("Abstract class")

    def __iadd__(self, module):
        """Attach the child 'module' at the next available attachment point"""
        raise NotImplementedError("Abstract class")

    def update(self, parent, pos, direction):
        """Update configuration for the module.

        This function should update the parent pointer and internal position of
        the module. The 'pos' argument is the central point where this module
        connects to parent. The 'direction' argument is a vector pointing in
        the direction the module should be attached at."""
        raise NotImplementedError("Abstract class")
