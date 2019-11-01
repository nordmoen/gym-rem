#!/usr/bin/env python

"""
Unittests for `ModularEnv`
"""

from gym_rem.morph import Servo
from gym_rem.envs import ModularEnv
import unittest


class TestModularEnv(unittest.TestCase):
    def test_collision(self):
        """Test that spawning colliding modules removes and returns offending
        modules"""
        env = ModularEnv()
        # The default Servo has three slots for new modules, however, one is
        # pointing down
        robot = Servo()
        robot += Servo()
        overflow = Servo()
        robot += overflow
        robot += Servo()
        env.reset(robot)
        self.assertLess(len(env.morphology), len(robot))

    def test_copy(self):
        """Test that holding on to a reference to a previous morphology is not
        equal to a new"""
        env = ModularEnv()
        a = Servo()
        env.reset(a)
        b = env.morphology
        env.reset(a)
        self.assertIsNotNone(b)


if __name__ == '__main__':
    unittest.main()
