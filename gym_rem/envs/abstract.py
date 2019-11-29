#!/usr/bin/env python

"""
Abstract modular environment.

Use this abstraction to implement new environments
"""

from collections import deque
from gym_rem.morph import Module
from pybullet_utils.bullet_client import BulletClient
import copy
import gym
import logging
import numpy as np
import os.path
import pybullet as pyb
import time

# Path to loadable assets
ASSET_PATH = os.path.join(os.path.dirname(__file__), "../../assets")


class ModularEnv(gym.Env):
    """Abstract modular environment"""

    metadata = {'render.modes': ['human']}

    def __init__(self):
        # Create logger for easy logging output
        self.log = logging.getLogger(self.__class__.__name__)
        # Create pybullet interfaces
        self.client = BulletClient(connection_mode=pyb.DIRECT)
        self._last_render = time.time()
        # Setup information needed for simulation
        self.dt = 1. / 240.
        self.client.setAdditionalSearchPath(ASSET_PATH)
        self._modules = {}
        self._joints = []
        # Stored for user interactions
        self.morphology = None
        # Used for user interaction:
        self._real_time = False
        # Run setup
        self.log.info("Creating modular environment")
        self.setup()

    def setup(self):
        """Helper method to initialize default environment"""
        # This is abstracted out from '__init__' because we need to do this
        # first time 'render' is called
        self.log.debug("Setting up simulation environment")
        self.client.resetSimulation()
        self.client.setGravity(0, 0, -9.81)
        self.client.setDefaultContactERP(0.9)
        # Physics engine parameters from:
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/scene_abstract.py#L74
        self.client.setPhysicsEngineParameter(fixedTimeStep=0.0165,
                                              numSolverIterations=5,
                                              numSubSteps=4)
        # Extract time step for sleep during rendering
        self.dt = self.client.getPhysicsEngineParameters()['fixedTimeStep']
        # Load ground plane for robots to walk on
        self.log.debug("Loading ground plane")
        self.plane_id = self.client.loadURDF('plane/plane.urdf')
        assert self.plane_id >= 0, "Could not load 'plane.urdf'"
        # Change dynamics parameters from:
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/scene_stadium.py#L33
        self.client.changeDynamics(self.plane_id, -1,
                                   lateralFriction=0.8, restitution=0.5)
        # Step world a few times for simulation to settle
        self.log.debug("Gym environment setup complete")

    def close(self):
        self.log.debug("Closing environment")
        self.client.disconnect()

    def reset(self, morphology=None, max_size=None):
        # Reset the environment by running all setup code again
        self.setup()
        # Reset internal state
        self._joints = []
        self._modules = {}
        # Check method arguments for simple logic errors
        if morphology is None:
            raise TypeError("Morphology cannot be 'None'!")
        if max_size is not None and max_size < 1:
            raise ValueError("'max_size' must be larger than 0")
        self.morphology = copy.deepcopy(morphology.root)
        # NOTE: We are using explicit queue handling here so that we can
        # ignore children of overlapping modules
        queue = deque([self.morphology])
        while len(queue) > 0:
            module = queue.popleft()
            assert isinstance(module, Module), "{} does not inherit\
                    from Module".format(module)
            # Spawn module in world
            m_id = module.spawn(self.client)
            # Check if the module overlaps
            aabb_min, aabb_max = self.client.getAABB(m_id)
            # Check overlapping modules
            overlap = self.client.getOverlappingObjects(aabb_min, aabb_max)
            # NOTE: An object always collides with it self
            overlapping_modules = any([u_id != m_id for u_id, _ in overlap])
            # Check against plane
            aabb_min, aabb_max = self.client.getAABB(self.plane_id)
            plane_overlap = self.client.getOverlappingObjects(aabb_min, aabb_max)
            overlapping_plane = any([u_id != self.plane_id
                                     for u_id, _ in plane_overlap])
            # If overlap is detected de-spawn module and continue
            if overlapping_modules or overlapping_plane:
                # Remove from simulation
                self.client.removeBody(m_id)
                # Remove from our private copy
                parent = module.parent
                del parent[module]
                continue
            # If not we continue to place it in the environment
            self._modules[module] = m_id
            # Add children to queue for processing
            queue.extend(module.children)
            # Handle joint information
            if module.joint is not None:
                self._joints.append((m_id, module.joint))
            # Create constraint so that modules are connected
            if module.parent is not None:
                parent_id = self._modules[module.parent]
                cid = self.client.createConstraint(parent_id, -1, m_id,
                                                   module.connection_id,
                                                   pyb.JOINT_FIXED,
                                                   module.connection_axis,
                                                   module.connection[0],
                                                   module.connection[1],
                                                   module.parent.orientation.T.as_quat(),
                                                   module.orientation.T.as_quat())
                # self.client.changeConstraint(cid, maxForce=10000.)
            # Check size constraints
            if max_size is not None and len(self._modules) >= max_size:
                # If we are above max desired spawn size drain queue and remove
                for module in queue:
                    parent = module.parent
                    del parent[module]
                break
        if not self._modules:
            # This is a bad sign and it can be difficult to debug because the
            # error occurs much later than here
            raise RuntimeError("No modules were spawned from morphology!")
        return self.observation()

    def step(self, action):
        assert action.shape[0] == len(self._joints), 'Action not equal to\
                number of joints'
        for (m_id, cfg), act in zip(self._joints, action):
            # Create a local copy so we can delete from it
            l_cfg = cfg.copy()
            l_cfg[l_cfg['target']] = act
            del l_cfg['target']
            self.client.setJointMotorControl2(m_id, **l_cfg)
        self.client.stepSimulation()
        return self.observation(), self.reward(), False, {}

    def observation(self):
        """Get observation from environment

        The default observation is `numpy.concatenate([positions, velocities,
        torques])` for all movable joints."""
        positions = []
        velocities = []
        torques = []
        for m_id, _ in self._joints:
            state = self.client.getJointState(m_id, 0)
            positions.append(state[0])
            velocities.append(state[1])
            torques.append(state[3])
        return np.concatenate([positions, velocities, torques])

    def reward(self):
        """Estimate current reward"""
        # Extract ID of root
        m_id = self._modules[self.morphology]
        # Get state of root link
        pos, _ = self.client.getBasePositionAndOrientation(m_id)
        # Extract position
        position = np.array(pos)
        # We are only interested in distance in (X, Y)
        position[-1] = 0.0
        # Return the root's distance from World center
        return np.linalg.norm(position)

    def render(self, mode="human"):
        info = self.client.getConnectionInfo()
        if info['connectionMethod'] != pyb.GUI and mode == 'human':
            self.log.debug("Enabling GUI rendering")
            self.close()
            self.log.debug("Starting GUI instance")
            self.client = BulletClient(connection_mode=pyb.GUI)
            self.setup()
            self._last_render = time.time()
            self.client.resetDebugVisualizerCamera(0.5, 50.0, -35.0, (0, 0, 0))
        elif info['connectionMethod'] == pyb.GUI:
            # Handle interaction with simulation
            self.handle_interaction()
            # Calculate time to sleep
            now = time.time()
            diff = now - self._last_render
            to_sleep = self.dt - diff
            if to_sleep > 0:
                time.sleep(to_sleep)
            self._last_render = now
        else:
            raise RuntimeError("Unknown pybullet mode ({}) or render mode ({})"
                               .format(info['connectionMethod'], mode))

    def handle_interaction(self):
        """Handle user interaction with simulation

        This function is called in the 'render' call and is only called if the
        GUI is visible"""
        keys = self.client.getKeyboardEvents()
        # If 'n' is pressed then we want to step the simulation
        next_key = ord('n')
        if next_key in keys and keys[next_key] & pyb.KEY_WAS_TRIGGERED:
            self.client.stepSimulation()
        # If space is pressed we start/stop real-time simulation
        space = ord(' ')
        if space in keys and keys[space] & pyb.KEY_WAS_TRIGGERED:
            self._real_time = not self._real_time
            self.client.setRealTimeSimulation(self._real_time)
        # if 'r' is pressed we restart simulation
        r = ord('r')
        if r in keys and keys[r] & pyb.KEY_WAS_TRIGGERED:
            self.reset(self.morphology)
