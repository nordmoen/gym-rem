#!/usr/bin/env python

"""
Unittests for morphology 'Module'
"""
from gym_rem.morph.exception import NoAvailable
from gym_rem.morph import Module
from gym_rem.utils import Rot
import numpy as np
import unittest


class TestMorpModule(unittest.TestCase):
    def test_pickle(self):
        """Ensure that the base morphology module can be pickled"""
        import pickle
        m = Module()
        data = pickle.dumps(m)
        del m
        _ = pickle.loads(data)

    def test_parent(self):
        """Ensure that abstract module has parent pointer"""
        m = Module()
        self.assertTrue(hasattr(m, 'parent'))
        self.assertIsNone(m.parent)

    def test_position(self):
        """Ensure that abstract module has position"""
        m = Module()
        self.assertTrue(hasattr(m, 'position'))
        self.assertTrue(np.allclose(m.position, np.zeros(3)))

    def test_orientation(self):
        """Ensure that abstract module has orientation"""
        m = Module()
        self.assertTrue(hasattr(m, 'orientation'))
        self.assertTrue(isinstance(m.orientation, Rot))

    def test_children(self):
        """Ensure that abstract module has method 'children'"""
        m = Module()
        self.assertTrue(hasattr(m, 'children'))
        self.assertListEqual(m.children, [])

    def test_len(self):
        """Ensure that abstract module supports 'len'"""
        m = Module()
        self.assertEqual(len(m), 1)

    def test_get(self):
        """Ensure that abstract module supports '[]' access"""
        m = Module()
        with self.assertRaises(NotImplementedError):
            m[None]

    def test_del(self):
        """Ensure that abstract module supports 'del []' access"""
        m = Module()
        with self.assertRaises(NotImplementedError):
            del m[None]

    def test_set(self):
        """Ensure that abstract module supports '[key] = value' access"""
        m = Module()
        with self.assertRaises(NotImplementedError):
            m[None] = None

    def test_iadd(self):
        """Ensure that abstract module supports '+=' access"""
        m = Module()
        with self.assertRaises(NoAvailable):
            m += None

    def test_available(self):
        """Ensure that abstract module has 'available' method"""
        m = Module()
        self.assertTrue(hasattr(m, 'available'))
        self.assertListEqual(m.available, [])

    def test_joint(self):
        """Ensure that abstract module has 'joint' method"""
        m = Module()
        self.assertTrue(hasattr(m, 'joint'))
        self.assertIsNone(m.joint)

    def test_update(self):
        """Ensure that abstract module has 'update' method"""
        m = Module()
        self.assertTrue(hasattr(m, 'update'))

    def test_spawn(self):
        """Ensure that abstract module has 'spawn' method"""
        m = Module()
        self.assertTrue(hasattr(m, 'spawn'))
        with self.assertRaises(NotImplementedError):
            m.spawn()


if __name__ == '__main__':
    unittest.main()
