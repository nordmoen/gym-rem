#!/usr/bin/env python
from setuptools import setup

setup(name='gym_rem',
      description="Modular robotics environment for OpenAi-Gym",
      version='0.3.0',
      keywords="modular robotics gym openai-gym",
      author="Jørgen Nordmoen and Frank Veenstra",
      author_email="jorgehn@ifi.uio.no",
      include_package_data=True,
      install_requires=['gym>=0.15', 'numpy>=1.17'],
      extras_require={
          '3D': ['pybullet>=2.6'],
          '2D': ['box2d-py>=2.3', 'pyglet>=1.3']
      },
      test_suite='tests')
