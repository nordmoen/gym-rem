#!/usr/bin/env python

"""
Example code to create random morphologies
"""

from gym_rem.envs import ModularEnv
from gym_rem.morph import Servo, Rect
import argparse
import numpy as np
import pickle
import tqdm


def _create_random(max_size):
    """Helper method to create random morphology"""
    # We always start with a static root
    root = Rect()
    # Select a random number of modules to attach
    size = np.random.randint(0, max_size)
    for _ in range(size):
        # Create a list of all available sites to place a module
        sites = []
        for module in root:
            sites.extend([(site, module) for site in module.available])
        # Select randomly the place to attach
        idx = np.random.randint(0, len(sites))
        conn, mod = sites[idx]
        # Select module randomly
        module = np.random.choice([Rect, Servo])
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
args = parser.parse_args()
# Set seed for reproducibility
np.random.seed(args.seed)
# Create default environment
env = ModularEnv()
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
    morph = _create_random(args.size)
    obs, _ = env.reset(morph)
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
