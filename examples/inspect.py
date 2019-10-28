#!/usr/bin/env python

"""
Tool to load morphology from 'pickle' file and inspect in GUI
"""
from gym_rem.envs import ModularEnv
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
except pickle.UnpicklingError:
    sys.exit("Could not parse morphology from file")
# Spawn morphology in environment
env = ModularEnv()
env.render()
env.reset(morph)
# Forever display morphology
while True:
    env.render()
