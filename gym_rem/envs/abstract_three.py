#!/usr/bin/env python

"""
Default 3D modular environment
"""

from collections import deque
from gym_rem.morph.three.abstract import Module3D
from gym_rem.utils.body import MultiBodyBuilder
from pybullet_utils.bullet_client import BulletClient
import copy
import gym
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
        # Create pybullet interfaces
        self.client = BulletClient(connection_mode=pyb.DIRECT)
        self._last_render = time.time()
        # Setup information needed for simulation
        self.dt = 1. / 240.
        self.client.setAdditionalSearchPath(ASSET_PATH)
        # Stored for user interactions
        self.morphology = None
        self.multi_id = None
        self._joint_ids = []
        self._joints = []
        # Used for user interaction:
        self._real_time = False
        # Run setup
        self.setup()

    def setup(self):
        """Helper method to initialize default environment"""
        # This is abstracted out from '__init__' because we need to do this
        # first time 'render' is called
        self.client.resetSimulation()
        self.client.setGravity(0, 0, -9.81)
        # Load ground plane for robots to walk on
        self.plane_id = self.client.loadURDF('plane/plane.urdf')
        assert self.plane_id >= 0, "Could not load 'plane.urdf'"
        # Change dynamics parameters from:
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/deep_mimic/env/pybullet_deep_mimic_env.py#L45
        self.client.changeDynamics(self.plane_id, -1, lateralFriction=0.9)
        # Extract time step for sleep during rendering
        self.dt = self.client.getPhysicsEngineParameters()['fixedTimeStep']

    def close(self):
        self.client.disconnect()

    def reset(self, morphology=None, max_size=None):
        # Reset the environment by running all setup code again
        self.setup()
        # Reset internal state
        self.morphology = None
        self.multi_id = None
        self._joint_ids = []
        self._joints = []
        # Check method arguments for simple logic errors
        if morphology is None:
            raise TypeError("Morphology cannot be 'None'!")
        if max_size is not None and max_size < 1:
            raise ValueError("'max_size' must be larger than 0")
        # NOTE: Utilize deep copy here to avoid interfering with morphology,
        # this means that if we want to do structural changes we can do so on
        # our own private instance. Which is nice for maximum size etc.
        self.morphology = copy.deepcopy(morphology.root)
        # Mapping from module to PyBullet ID
        spawned_ids = {}
        # NOTE: We are using explicit queue handling here so that we can
        # ignore children of overlapping modules
        queue = deque([self.morphology.root])
        while len(queue) > 0:
            module = queue.popleft()
            assert isinstance(module, Module3D), "{} does not inherit\
                from 3D Module".format(type(module))
            # Spawn module in world
            m_id = module.spawn(self.client)
            # Check if the module overlaps
            aabb_min, aabb_max = self.client.getAABB(m_id)
            # Check overlapping modules
            overlap = self.client.getOverlappingObjects(aabb_min, aabb_max)
            # NOTE: An object always collides with it self
            overlapping_modules = False
            if overlap:
                overlapping_modules = any([u_id != m_id for u_id, _ in overlap])
            # Check against plane
            aabb_min, aabb_max = self.client.getAABB(self.plane_id)
            plane_overlap = self.client.getOverlappingObjects(aabb_min, aabb_max)
            overlapping_plane = False
            if plane_overlap:
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
            # Add children to queue for processing
            queue.extend(module.children)
            # Add ID to spawned IDs so that we can remove them later
            spawned_ids[module] = m_id
            # Add joint if present
            if module.joint:
                self._joints.append(module.joint)
            # Check size constraints
            if max_size is not None and len(spawned_ids) >= max_size:
                # If we are above max desired spawn size drain queue and remove
                for module in queue:
                    parent = module.parent
                    del parent[module]
                break
        # Create multi body builder and instantiate with morphological tree and
        # module IDs
        builder = MultiBodyBuilder(self.morphology, spawned_ids, self.client,
                                   flags=pyb.URDF_USE_SELF_COLLISION)
        # Before spawning multi body we remove all modules so far
        # NOTE: We must do that here after all modules have been spawned to not
        # interfere with collision detection
        for m_id in spawned_ids.values():
            self.client.removeBody(m_id)
        # Instantiate the builder creating a multi body from the morphology
        multi_id = builder.build()
        assert multi_id >= 0, "Could not spawn multibody!"
        self.multi_id = multi_id
        for jid in range(self.client.getNumJoints(self.multi_id)):
            j_info = self.client.getJointInfo(self.multi_id, jid)
            if j_info[2] != pyb.JOINT_FIXED:
                self._joint_ids.append(jid)
        self.observation_space = gym.spaces.Box(-100., 100.,
                                                shape=(3 * len(self._joints),))
        lows = np.repeat(-1.57, len(self._joints))
        self.action_space = gym.spaces.Box(lows, -1. * lows)
        return self.observation()

    def act(self, action):
        """Helper function to apply actions to robot"""
        assert action.shape[0] == len(self._joint_ids), 'Action not equal to\
            number of joints'
        for j_id, cfg, act in zip(self._joint_ids, self._joints, action):
            # Create a local copy so we can delete from it
            l_cfg = cfg.copy()
            # If joint should be limited we check that here
            if 'limit' in l_cfg:
                act = max(l_cfg['limit'][0], min(l_cfg['limit'][1], act))
                del l_cfg['limit']
            # Extract type of 'target' for joint
            l_cfg[l_cfg['target']] = act
            del l_cfg['target']
            self.client.setJointMotorControl2(self.multi_id, j_id, **l_cfg)

    def step(self, action):
        self.act(action)
        self.client.stepSimulation()
        return self.observation(), self.reward(), False, {}

    def observation(self):
        """Get observation from environment

        The default observation is `numpy.concatenate([positions, velocities,
        torques])` for all movable joints."""
        # If the body does not have any movable joints:
        if not self._joint_ids:
            return np.array([])
        positions = []
        velocities = []
        torques = []
        states = self.client.getJointStates(self.multi_id, self._joint_ids)
        for state in states:
            positions.append(state[0])
            velocities.append(state[1])
            torques.append(state[3])
        return np.concatenate([positions, velocities, torques])

    def reward(self):
        """Estimate current reward"""
        # Get state of root link
        pos, _ = self.client.getBasePositionAndOrientation(self.multi_id)
        # Extract position
        position = np.array(pos)
        # We are only interested in distance in (X, Y)
        position[-1] = 0.0
        # Return the root's distance from World center
        return np.linalg.norm(position)

    def render(self, mode="human"):
        info = self.client.getConnectionInfo()
        if info['connectionMethod'] != pyb.GUI and mode == 'human':
            # Close current simulation and start new with GUI
            self.close()
            self.client = BulletClient(connection_mode=pyb.GUI)
            if self.morphology is not None:
                self.reset(self.morphology)
            else:
                self.setup()
            # Configure viewport
            self.client.configureDebugVisualizer(pyb.COV_ENABLE_GUI, 0)
            self.client.resetDebugVisualizerCamera(0.5, 50.0, -35.0, (0, 0, 0))
            # Setup timing for correct sleeping when rendering
            self._last_render = time.time()
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
            real_time = self._real_time
            self.client.setRealTimeSimulation(False)
            self.reset(self.morphology)
            self.client.setRealTimeSimulation(real_time)
