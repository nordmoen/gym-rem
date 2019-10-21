#!/usr/bin/env python

"""
Test module for servo.py
"""
from gym_rem.morph import Servo
from gym_rem.morph.servo import Connection
import numpy as np
import unittest


class TestServo(unittest.TestCase):
    def test_pickle(self):
        """Ensure that servo can be pickled"""
        import pickle
        m = Servo()
        data = pickle.dumps(m)
        del m
        _ = pickle.loads(data)

    def test_update(self):
        """Ensure that 'update' functions correctly"""
        a = Servo()
        b = Servo()
        a += b
        self.assertEqual(b.parent, a)
        self.assertTrue(np.allclose(b.position, np.array([0., 0.02, 0.01])))

    def test_children(self):
        """Test that children property works"""
        a = Servo()
        self.assertListEqual(a.children, [])
        b = Servo()
        self.assertIsNone(b.parent)
        a += b
        self.assertListEqual(a.children, [b])

    def test_iter(self):
        """Ensure that iteration works"""
        a = Servo()
        b = Servo()
        c = Servo()
        d = Servo()
        a += b
        b += c
        a += d
        modules = [m for m in a]
        self.assertEqual(modules[0], a)
        self.assertEqual(modules[1], b)
        # NOTE: Current implementation is breadth first
        self.assertEqual(modules[2], d)
        self.assertEqual(modules[3], c)

    def test_available(self):
        """Ensure that the 'available' method returns the correct points"""
        # Test default orientation
        m = Servo()
        self.assertListEqual(m.available, [c for c in Connection])
        # Test 90 deg
        m = Servo(1)
        self.assertListEqual(m.available,
                             [Connection.z_plus, Connection.y_minus,
                              Connection.x_plus])
        m = Servo(2)
        self.assertListEqual(m.available,
                             [Connection.y_plus, Connection.y_minus,
                              Connection.x_plus])
        m = Servo(3)
        self.assertListEqual(m.available,
                             [Connection.y_plus, Connection.z_plus,
                              Connection.x_plus])

    def test_len(self):
        """Test that superclass '__len__' functions correctly"""
        a = Servo()
        self.assertEqual(len(a), 1)
        a += Servo()
        self.assertEqual(len(a), 2)
        a.children[0] += Servo()
        self.assertEqual(len(a), 3)
        a += Servo()
        self.assertEqual(len(a), 4)
        a.children[0].children[0] += Servo()
        self.assertEqual(len(a), 5)

    def test_root(self):
        """Test that superclass 'root' functions correctly"""
        a = Servo()
        self.assertEqual(a.root, a)
        b = Servo()
        a += b
        self.assertEqual(b.root, a)
        c = Servo()
        b += c
        self.assertEqual(c.root, a)
        d = Servo()
        d += a
        self.assertEqual(a.root, d)
        self.assertEqual(c.root, d)


if __name__ == '__main__':
    unittest.main()
