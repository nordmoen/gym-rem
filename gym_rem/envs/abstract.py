#!/usr/bin/env python

"""
Abstract modular environment.

Use this abstraction to implement new environments
"""

from gym_rem.morph import Servo, Module
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

    metadata = {'render.modes': ['human'], 'video.frames_per_second': 60}

    def __init__(self):
        # Create logger for easy logging output
        self.log = logging.getLogger(self.__class__.__name__)
        # Create pybullet interfaces
        self.client = pyb.connect(pyb.DIRECT)
        self._last_render = time.time()
        # Setup information needed for simulation
        self.dt = 1. / 240.
        pyb.setAdditionalSearchPath(ASSET_PATH)
        self._modules = {}
        self._joints = []
        self._morphology = None
        # Run setup
        self.log.info("Creating modular environment")
        self.setup()

    def setup(self):
        """Helper method to initialize default environment"""
        # This is abstracted out from '__init__' because we need to do this
        # first time 'render' is called
        self.log.debug("Setting up simulation environment")
        pyb.resetSimulation()
        pyb.setGravity(0, 0, -9.81)
        # Extract time step for sleep during rendering
        self.dt = pyb.getPhysicsEngineParameters()['fixedTimeStep']
        # Load ground plane for robots to walk on
        self.log.debug("Loading ground plane")
        self.plane_id = pyb.loadURDF('plane/plane.urdf')
        assert self.plane_id >= 0, "Could not load 'plane.urdf'"
        # Setup reset point so that we can quickly reset simulation
        self.reset_id = pyb.saveState()
        assert self.reset_id >= 0, "Could not create save state in memory"
        self.log.debug("Gym environment setup complete")

    def close(self):
        self.log.debug("Closing environment")
        pyb.disconnect(self.client)

    def reset(self, morphology=None):
        # To reset simulation we first need to remove any bodies
        for m_id in self._modules.values():
            pyb.removeBody(m_id)
        pyb.restoreState(stateId=self.reset_id)
        overlaps = []
        if morphology is None:
            raise TypeError("Morphology cannot be 'None'!")
        # Reset internal state
        self._joints = []
        self._modules = {}
        self._morphology = morphology.root
        # NOTE: We are using explicit queue handling here so that we can
        # ignore children of overlapping modules
        queue = [self._morphology]
        while len(queue) > 0:
            module = queue.pop()
            assert isinstance(module, Module), "{} does not inherit\
                    from Module".format(module)
            # Spawn module in world
            m_id = self.spawn(module)
            # Check if the module overlaps
            # TODO Only check local overlap, no need to check the whole
            # world
            overlap = pyb.getOverlappingObjects([-10, -10, 0],
                                                [10, 10, 10])
            if m_id in overlap:
                overlaps.append(module)
                pyb.removeBody(m_id)
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
                parent_frame = module.connection[0]
                child_frame = module.connection[1]
                pyb.createConstraint(parent_id, -1, m_id, 0,
                                     pyb.JOINT_FIXED,
                                     (-1, 0, 0),
                                     parent_frame,
                                     child_frame)
                # pyb.setCollisionFilterPair(parent_id, m_id, 0, 0, 0)
        return self.observation(), overlaps

    def step(self, action):
        assert action.shape[0] == len(self._joints), 'Action not equal to\
                number of joints'
        for (m_id, cfg), act in zip(self._joints, action):
            # Create a local copy so we can delete from it
            l_cfg = cfg.copy()
            idx = l_cfg['jointIndex']
            del l_cfg['jointIndex']
            mode = l_cfg['controlMode']
            del l_cfg['controlMode']
            pyb.setJointMotorControl2(m_id, idx, mode,
                                      targetPosition=act,
                                      **l_cfg)
        pyb.stepSimulation()
        return self.observation(), self.reward(), False, {}

    def observation(self):
        """Get observation from environment

        The default observation is `numpy.concatenate([positions, velocities,
        torques])` for all movable joints."""
        positions = []
        velocities = []
        torques = []
        for m_id, _ in self._joints:
            state = pyb.getJointState(m_id, 0)
            positions.append(state[0])
            velocities.append(state[1])
            torques.append(state[3])
        return np.concatenate([positions, velocities, torques])

    def reward(self):
        """Estimate current reward"""
        # Extract ID of root
        m_id = self._modules[self._morphology]
        # Get state of root link
        state = pyb.getLinkState(m_id, 0)
        # Extract position
        position = np.array(state[0])
        # We are only interested in distance in (X, Y)
        position[-1] = 0.0
        # Return the root's distance from World center
        return np.linalg.norm(position)

    def spawn(self, module):
        """Spawn the given module in the simulation"""
        if isinstance(module, Servo):
            orient = pyb.getQuaternionFromEuler(module.orientation)
            return pyb.loadURDF('servo/Servo.urdf',
                                basePosition=module.position,
                                baseOrientation=orient)
        else:
            raise TypeError("Unknown module type: {} ({}) can't spawn"
                            .format(type(module), module))

    def render(self, mode="human"):
        info = pyb.getConnectionInfo(self.client)
        if info['connectionMethod'] != pyb.GUI and mode == 'human':
            self.log.debug("Enabling GUI rendering")
            self.close()
            self.log.debug("Starting GUI instance")
            self.client = pyb.connect(pyb.GUI)
            self.log.debug("Setting up GUI instance")
            self.setup()
            self._last_render = time.time()
            pyb.resetDebugVisualizerCamera(0.5, 50.0, -35.0, (0, 0, 0))
        elif info['connectionMethod'] == pyb.GUI:
            now = time.time()
            diff = now - self._last_render
            to_sleep = self.dt - diff
            if to_sleep > 0:
                time.sleep(to_sleep)
        else:
            raise RuntimeError("Unknown pybullet mode ({}) or render mode ({})"
                               .format(info['connectionMethod'], mode))
