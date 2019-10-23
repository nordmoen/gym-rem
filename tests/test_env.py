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
        _, rest = env.reset(robot)
        self.assertListEqual(rest, [overflow])


if __name__ == '__main__':
    unittest.main()
