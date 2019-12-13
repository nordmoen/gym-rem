#!/usr/bin/env python

"""
Helper file to benchmark PyBullet and modular robotics
"""

import argparse
import gym
import gym_rem
import numpy as np
import timeit


# Create and parse command line arguments
parser = argparse.ArgumentParser("gym_rem:benchmark")
parser.add_argument('--number', '-n', type=int, default=1000,
                    help="Number of iterations per method")
parser.add_argument('--length', '-l', type=int, default=50,
                    help="Number of modules to create")
parser.add_argument('--env', choices=['2d', '3d'], default='3d',
                    help="Utilize 2D or 3D environment")
args = parser.parse_args()
# Create environment
if args.env == '2d':
    env = gym.make('ModularLocomotion2D-v0')
    from gym_rem.morph.two.servo import Servo, Connection
elif args.env == '3d':
    env = gym.make('ModularLocomotion3D-v0')
    from gym_rem.morph.three.servo import Servo, Connection
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
if args.env == '2d':
    create_num = timeit.timeit("_ = gym.make('ModularLocomotion2D-v0')",
                               setup="import gym;import gym_rem",
                               number=20)
elif args.env == '3d':
    create_num = timeit.timeit("_ = gym.make('ModularLocomotion3D-v0')",
                               setup="import gym;import gym_rem",
                               number=20)
create_num /= 20.0
print("'ModularEnv()':\t\t\t{:.3f} milliseconds".format(create_num * 1000.))
copy_num = timeit.timeit("copy.deepcopy(morph)", setup="import copy",
                         number=num, globals=glob)
copy_num /= fnum
print("'copy.deepcopy(...)':\t\t{:.3f} milliseconds".format(copy_num * 1000.))
reset_num = timeit.timeit("env.reset(morph)", number=num, globals=glob)
reset_num /= fnum
print("'env.reset(...)':\t\t{:.3f} milliseconds".format(reset_num * 1000.))
obs_num = timeit.timeit("env.observation()", number=num, globals=glob)
obs_num /= fnum
print("'env.observation()':\t\t{:.3f} milliseconds".format(obs_num * 1000.))
rew_num = timeit.timeit("env.reward()", number=num, globals=glob)
rew_num /= fnum
print("'env.reward()':\t\t\t{:.3f} milliseconds".format(rew_num * 1000.))
act_num = timeit.timeit("env.act(action)", number=num, globals=glob)
act_num /= fnum
print("'env.act(...)':\t\t\t{:.3f} milliseconds".format(act_num * 1000.))
step_num = timeit.timeit("env.step(action)", number=num, globals=glob)
step_num /= fnum
print("'env.step(...)':\t\t{:.3f} milliseconds".format(step_num * 1000.))
print("Simulation speed-up:\t\t~{:.1f}x real-time".format(dt / step_num))
