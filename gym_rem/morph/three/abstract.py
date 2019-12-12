#!/usr/bin/env python

"""
Abstract 3D module
"""
from gym_rem.morph import Module


class Module3D(Module):
    """Abstract 3D module"""

    @property
    def joint(self):
        """
        Get the joint configuration as a dict.

        The joint configuration is a leaky abstraction used for movable joints.
        Subclasses should return a dict with joint configuration according to
        pybyllet, supported items are 'controlMode' (required), 'jointIndex'
        (required), 'target' (required, one of 'targetPosition',
        'targetVelocity' or 'force') 'positionGain' (optional), 'velocityGain'
        (optional) and 'maxVelocity' (optional).

        Non-movable joints should return None.
        """
        return None

    def spawn(self, client):
        """Spawn the module in the physics simulation"""
        raise NotImplementedError("Not supported")
