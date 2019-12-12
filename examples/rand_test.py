#!/usr/bin/env python

"""
Example code to create random morphologies
"""

import argparse
import gym
import gym_rem
import numpy as np
import pickle
import tqdm


def _create_random(max_size, type='3d'):
    """Helper method to create random morphology"""
    if type == '3d':
        from gym_rem.morph.three import Servo, Rect
        initial = Rect
        choices = [Rect, Servo]
    else:
        from gym_rem.morph.two import Servo, Rect
        initial = Rect
        choices = [Rect, Servo]
    # We always start with a static root
    root = initial()
    # Select a random number of modules to attach
    size = np.random.randint(1, max_size)
    # size = max_size
    for _ in range(size):
        # Create a list of all available sites to place a module
        sites = []
        for module in root:
            sites.extend([(site, module) for site in module.available])
        # Select randomly the place to attach
        idx = np.random.randint(0, len(sites))
        conn, mod = sites[idx]
        # Select module randomly
        module = np.random.choice(choices)
        # Update morphology with new random module
        mod[conn] = module()
    return root


# Create a quick and dirty argument parser
parser = argparse.ArgumentParser("gym_rem:random")
parser.add_argument('--number', '-n', type=int, default=1000,
                    help="Number of random morphologies to create")
parser.add_argument('--size', '-s', type=int, default=50,
                    help="Maximum size of a morphology")
parser.add_argument('--seconds', '-sec', type=int, default=10,
                    help="Number of simulated seconds per morphology")
parser.add_argument('--gui', '-g', action='store_true',
                    help="Show GUI")
parser.add_argument('--quiet', '-q', action='store_true',
                    help="Do not show progress")
parser.add_argument('--seed', type=int, default=1234,
                    help="Random seed")
parser.add_argument('--env', choices=['2d', '3d'], default='3d',
                    help="Use 2D or 3D environment")
args = parser.parse_args()
# Set seed for reproducibility
np.random.seed(args.seed)
# Create default environment
if args.env == '3d':
    env = gym.make('ModularLocomotion3D-v0')
else:
    env = gym.make('ModularLocomotion2D-v0')
if args.gui:
    env.render()
# Get number of steps per simulated seconds
dt = env.dt
# Setup progressbar
prog = tqdm.trange(args.number, desc="Testing morphology", leave=False,
                   disable=args.quiet, unit="morphology")
# For each morphology
for i in prog:
    # Create random morphology and reset environment
    morph = _create_random(args.size, args.env)
    obs = env.reset(morphology=morph)
    # Simulate the morphology to ensure that it doesn't do anything weired
    rew = 0.0
    for _ in range(int(args.seconds / dt)):
        _, rew, _, _ = env.step(np.zeros(int(obs.shape[0] / 3)))
        if args.gui:
            env.render()
    # When done check latest reward to see distance moved
    if rew > 0.1:
        prog.write("Morphology {:d} moved: {:.2f}".format(i, rew))
        with open('bad_morph_{:d}.pickle'.format(i), 'wb') as fil:
            pickle.dump(morph, fil)
