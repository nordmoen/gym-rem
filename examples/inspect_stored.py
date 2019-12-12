#!/usr/bin/env python

"""
Tool to load morphology from 'pickle' file and inspect in GUI
"""
import gym
from gym_rem.morph import Module2D, Module3D
import argparse
import pickle
import sys


# Create quick and dirty command line parser
parser = argparse.ArgumentParser("gym_rem:inspect")
parser.add_argument('file', type=argparse.FileType('rb'),
                    help="File to read morphology from")
args = parser.parse_args()
try:
    morph = pickle.load(args.file)
    # Pock all the modules
    morph.root.update_children()
except pickle.UnpicklingError:
    sys.exit("Could not parse morphology from file")
# Spawn morphology in environment
if isinstance(morph, Module2D):
    env = gym.make('ModularLocomotion2D-v0')
elif isinstance(morph, Module3D):
    env = gym.make('ModularLocomotion3D-v0')
env.render()
env.reset(morphology=morph)
env.morphology.root.update_children()
# Forever display morphology
while True:
    env.render()
