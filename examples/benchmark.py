#!/usr/bin/env python

"""
Helper file to benchmark PyBullet and modular robotics
"""

from gym_rem.envs import ModularEnv
from gym_rem.morph.servo import Servo, Connection
import argparse
import numpy as np
import timeit


# Create and parse command line arguments
parser = argparse.ArgumentParser("gym_rem:benchmark")
parser.add_argument('--number', '-n', type=int, default=1000,
                    help="Number of iterations per method")
parser.add_argument('--length', '-l', type=int, default=50,
                    help="Number of modules to create")
args = parser.parse_args()
# Create environment
env = ModularEnv()
# Create robot
root = Servo()
current = root
for _ in range(args.length - 1):
    tmp = Servo()
    current[Connection.x_plus] = tmp
    current = tmp
# Create global variables to pass to testing
glob = {'env': env, 'action': np.zeros(args.length), 'morph': root}
# Access timestep of PyBullet
dt = env.dt
# Run timings
num = args.number
fnum = float(num)
reset_num = timeit.timeit("env.reset(morph)", number=num, globals=glob)
reset_num /= fnum
print("'env.reset(...)':\t\t{:.3f} milliseconds".format(reset_num * 1000.))
obs_num = timeit.timeit("env.observation()", number=num, globals=glob)
obs_num /= fnum
print("'env.observation()':\t\t{:.3f} milliseconds".format(obs_num * 1000.))
rew_num = timeit.timeit("env.reward()", number=num, globals=glob)
rew_num /= fnum
print("'env.reward()':\t\t\t{:.3f} milliseconds".format(rew_num * 1000.))
step_num = timeit.timeit("env.step(action)", number=num, globals=glob)
step_num /= fnum
print("'env.step(...)':\t\t{:.3f} milliseconds".format(step_num * 1000.))
print("Simulation speed-up:\t\t~{:.1f}x real-time".format(dt / step_num))
