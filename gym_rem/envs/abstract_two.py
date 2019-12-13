#!/usr/bin/env python

"""
Default 2D modular environment
"""

from collections import deque
from gym_rem.morph.two import Module2D, Rect, Servo
import Box2D
import copy
import gym
import math
import numpy as np

# Static constants for environment
VIEWPORT_W = 768
VIEWPORT_H = 576
GROUND_HEIGHT = 10
# Fixture definition for ground
GROUND_FD = Box2D.b2FixtureDef(
    shape=Box2D.b2PolygonShape(box=(VIEWPORT_W * 100, GROUND_HEIGHT)))
# Fixture definitions for bodies
CUBE_FD = Box2D.b2FixtureDef(
    shape=Box2D.b2PolygonShape(box=(20, 20)),
    density=0.00025)
PLATE_FD = Box2D.b2FixtureDef(
    shape=Box2D.b2PolygonShape(box=(4, 20)),
    density=0.000103)
# Enable debug if debug drawing is desired
DEBUG = True


class ModularEnv2D(gym.Env):
    """Default 2D modular environment"""

    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, dt=1./60.):
        # Setup world and potentially viewport
        self.world = None
        self.viewer = None
        # Viewer parameters
        self._scroll = 1.
        self._view = np.array([VIEWPORT_W/2, 0.])
        # Setup morphology
        self.morphology = None
        self._joints = {}  # Mapping from module.joint to Box2D joint
        self._bodies = []
        self._module_map = {}  # Mapping from module to slice of bodies
        self.ground = None
        # Setup simulation parameters
        self.dt = dt
        self._setup()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def _on_scroll(self, x, y, scroll_x, scroll_y):
        self._scroll += 0.1 * scroll_y
        self._scroll = min(3.5, max(self._scroll, 0.2))

    def _on_drag(self, x, y, dx, dy, buttons, modifiers):
        if buttons:
            self._view += np.array([dx, 0.])

    def _on_key(self, symb, modif):
        from pyglet.window import key
        if symb == key._0 and modif & key.MOD_ACCEL:
            self._scroll = 1.
            # Reset with body in center
            x_pos = VIEWPORT_W / 2. + self._bodies[0].position.x
            self._view = np.array([x_pos, 0.])
        elif symb == key.S:
            self.world.Step(self.dt, 30*6, 30*2)
        elif symb == key.R:
            self.reset(self.morphology)
        elif symb == key.SPACE:
            for _ in range(int(1. / self.dt)):
                self.world.Step(self.dt, 30*6, 30*2)

    def render(self, mode='human'):
        from gym.envs.classic_control import rendering
        if self.viewer is None:
            self.viewer = rendering.Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.window.set_caption("Robotics, Evolution and Modularity - OpenAI Gym")
            self.viewer.window.on_mouse_scroll = self._on_scroll
            self.viewer.window.on_mouse_drag = self._on_drag
            self.viewer.window.on_key_press = self._on_key
        # First we render ground
        ground_path = [np.array(v) * self._scroll + self._view
                       for v in self.ground.fixtures[0].shape.vertices]
        self.viewer.draw_polygon(ground_path, color=(0.0, 0.26, 0.15))
        # Draw spawned bodies
        for body in self._bodies:
            for fixture in body.fixtures:
                trans = fixture.body.transform
                path = [trans * v * self._scroll + self._view
                        for v in fixture.shape.vertices]
                self.viewer.draw_polygon(path, color=body.color1)
                path.append(path[0])
                self.viewer.draw_polyline(path, color=body.color2,
                                          linewidth=2. * self._scroll)
                # Add a yellow line at the point where this connects to other
                # modules for easier visualization of connections
                if hasattr(body, 'draw_connection') and body.draw_connection:
                    self.viewer.draw_polyline(path[0:2], color=(1.0, 0.91, 0.15),
                                              linewidth=2 * self._scroll)
                if DEBUG:
                    self.viewer.draw_polyline(path[0:2], color=(1.0, 0.0, 0.0),
                                              linewidth=2 * self._scroll)
        if DEBUG:
            # Only draw the following in debug mode
            for joint in self.world.joints:
                # if type(joint) is Box2D.b2WeldJoint:
                filled = type(joint) is Box2D.b2WeldJoint
                pos1 = np.array(joint.anchorA) * self._scroll + self._view
                t1 = rendering.Transform(translation=pos1)
                self.viewer.draw_circle(radius=2 * self._scroll,
                                        color=(0, 1, 0),
                                        filled=filled).add_attr(t1)
                self.viewer.draw_line(pos1, np.array(joint.bodyA.position) * self._scroll + self._view)
                pos2 = np.array(joint.anchorB) * self._scroll + self._view
                t2 = rendering.Transform(translation=pos1)
                self.viewer.draw_circle(radius=2 * self._scroll,
                                        color=(0, 0, 1),
                                        filled=filled).add_attr(t2)
                self.viewer.draw_line(pos2, np.array(joint.bodyB.position) * self._scroll + self._view)
        # Return rendering result
        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def _setup(self):
        """Helper method to initialize default environment"""
        self.world = Box2D.b2World()
        self.ground = self.world.CreateStaticBody(
            position=(VIEWPORT_W / 2., 0.0),
            fixtures=GROUND_FD)

    def _spawn(self, module):
        """Helper method to spawn 2D module inside Box2D"""
        bodies = []
        joints = []
        position_correction = np.array([0.0, GROUND_HEIGHT + 0.1])
        axis, angle = module.orientation.as_axis()
        # angle = module.orientation.as_euler()[0]
        angle = axis[1] * angle * -1.
        if isinstance(module, Rect):
            # Spawn rectangle module
            body = self.world.CreateDynamicBody(
                position=module.position[::2] + position_correction,
                angle=angle,
                fixtures=CUBE_FD)
            body.color1 = (0.23, 0.32, 0.55)
            body.color2 = (0, 0, 0)
            body.draw_connection = True
            bodies.append(body)
        elif isinstance(module, Servo):
            # Spawn rectangle module
            body = self.world.CreateDynamicBody(
                position=module.position[::2] + position_correction,
                angle=angle,
                fixtures=CUBE_FD)
            body.color1 = (0.13, 0.57, 0.55)
            body.color2 = (0, 0, 0)
            bodies.append(body)
            # Spawn plate
            plate_corr = module.orientation.rotate(np.array([-29., 0., 0]))
            plate = self.world.CreateDynamicBody(
                position=module.position[::2] + position_correction + plate_corr[::2],
                angle=angle,
                fixtures=PLATE_FD)
            plate.color1 = (0.99, 0.90, 0.15)
            plate.color2 = (0, 0, 0)
            bodies.append(plate)
            # Create joint between bodies
            joint = Box2D.b2RevoluteJointDef(
                bodyA=body,
                bodyB=plate,
                localAnchorA=(0, 0),
                localAnchorB=-plate_corr[::2] if math.isclose(angle, 0.0) else (29., 0.),
                enableMotor=True,
                enableLimit=True,
                maxMotorTorque=1000.8,
                lowerAngle=-1.57,
                upperAngle=1.57,
                collideConnected=False)
            joint = self.world.CreateJoint(joint)
            joints.append(joint)
        else:
            raise NotImplementedError("Unknown module type: '{!s}'"
                                      .format(type(module)))
        # Do collision detection
        self.world.contactManager.FindNewContacts()
        self.world.contactManager.Collide()
        if any(map(lambda c: c.touching, self.world.contacts)):
            # Collision detected, remove bodies
            for joint in joints:
                self.world.DestroyJoint(joint)
            for body in bodies:
                self.world.DestroyBody(body)
            return False
        # No collision detected add bodies
        self._bodies.extend(bodies)
        self._joints.extend(joints)
        self._module_map[module] = bodies
        return True

    def reset(self, morphology, max_size=None):
        # Reset world
        self._setup()
        # Reset internal state
        self.morphology = None
        self._joints = []
        self._bodies = []
        # Check arguments for validity
        if max_size is not None and max_size < 1:
            raise ValueError("'max_size' must be larger than 0")
        if not isinstance(morphology, Module2D):
            raise ValueError("Morphology is not expected type,\
                expected 'Module2D' found: '{!s}'".format(type(morphology)))
        # NOTE: Utilize deep copy here to avoid interfering with morphology,
        # this means that if we want to do structural changes we can do so on
        # our own private instance. Which is nice for maximum size etc.
        self.morphology = copy.deepcopy(morphology.root)
        # NOTE: We use explicit queue handling here so that we can control the
        # queue to remove and insert at will
        queue = deque([self.morphology.root])
        spawned = 0
        while queue:
            module = queue.popleft()
            assert isinstance(module, Module2D), "{} does not inherit\
                from 2D module".format(type(module))
            # Try to spawn the module in the environment
            if not self._spawn(module):
                # If we could not spawn it we delete it from its parent and
                # continue
                if module.parent:
                    parent = module.parent
                    del parent[module]
                continue
            if module.parent:
                parent_body = self._module_map[module.parent][0]
                child_body = self._module_map[module][-1]
                if math.isclose(parent_body.angle, 0.0):
                    parent_orient = module.parent.orientation.rotate(module.connection[0])
                else:
                    parent_orient = module.connection[0]
                if math.isclose(child_body.angle, 0.0):
                    child_orient = module.orientation.rotate(module.connection[1])
                else:
                    child_orient = module.connection[1]
                joint = self.world.CreateWeldJoint(
                    bodyA=parent_body,
                    bodyB=child_body,
                    localAnchorA=parent_orient[::2],
                    localAnchorB=child_orient[::2])
            # Remember to count number of modules spawned
            spawned += 1
            # Since it spawned successfully we add its children
            queue.extend(module.children)
            # Ensure that we don't spawn more modules than desired
            if max_size is not None and spawned >= max_size:
                for module in queue:
                    parent = module.parent
                    del parent[module]
                break
        if not self._bodies:
            raise RuntimeError("No bodies were spawned from morphology")
        # Setup observation - and action space
        self.observation_space = gym.spaces.Box(-100., 100.,
                                                shape=(3 * len(self._joints),))
        lows = np.repeat(-1.57, len(self._joints))
        self.action_space = gym.spaces.Box(lows, lows * -1.)
        # Return observation
        return self.observation()

    def observation(self):
        """Calculate the current observation"""
        # If no joints are present we don't return anything
        if not self._joints:
            return np.array([])
        # Return position, velocity and force/torque for each joint
        positions = []
        velocities = []
        torques = []
        for joint in self._joints:
            positions.append(joint.angle)
            velocities.append(joint.speed)
            torques.append(joint.GetMotorTorque(self.dt))
        return np.concatenate([positions, velocities, torques])

    def reward(self):
        """Calculate the current reward"""
        # Assume the initial body is the root
        root = self._bodies[0]
        position = np.array(root.position)
        # We are only interested in distance along the X-axis
        position[1] = 0.0
        # Return the distance from the initial position
        # NOTE: We scale the reward by 10 to get values that feel a bit more
        # correct for the 2D case since positions are in pixels
        return np.linalg.norm(position) / 10.

    def act(self, action):
        """Helper method to apply desired action to joints"""
        assert self.action_space.contains(action)
        for a, joint in zip(action, self._joints):
            if joint.angle == a:
                joint.motorSpeed = 0.0
            else:
                joint.motorSpeed = 10.8 * np.sign(a)

    def step(self, action):
        self.act(action)
        self.world.Step(self.dt, 6*30, 2*30)
        return self.observation(), self.reward(), False, {}
