#!/usr/bin/env python

"""
Utility to fuse individual models into a 'MultiBody' in PyBullet
"""
from gym_rem.morph import Servo
import copy
import numpy as np
import pybullet as pyb


GEOM_NAME = {
        pyb.GEOM_BOX: "Box",
        pyb.GEOM_SPHERE: "Sphere",
        pyb.GEOM_MESH: "Mesh",
        pyb.GEOM_CYLINDER: "Cylinder",
        pyb.GEOM_CAPSULE: "Capsule",
        pyb.GEOM_PLANE: "Plane"}
# Link cache used to cache link elements to avoid problems when a body is
# removed
LINK_CACHE = {}


class Collision(object):
    """PyBullet collision object"""
    def __init__(self, position, orientation, type, radius=0.5,
                 extents=np.zeros(3), length=1., mesh_filename='mesh',
                 mesh_scale=np.ones(3)):
        self.position = position
        self.orientation = orientation
        self.type = type
        self.radius = radius
        self.extents = 0.5 * extents
        self.length = length
        self.mesh_filename = mesh_filename
        self.mesh_scale = mesh_scale

    def __repr__(self):
        return "Collision({!s})".format(GEOM_NAME[self.type])

    @staticmethod
    def from_shape(shape):
        """Create a Collision object from a PyBullet CollisionShapeData
        object"""
        typ = shape[2]
        dims = np.array(shape[3])
        filename = shape[4].decode('utf-8')
        pos = shape[5]
        orient = shape[6]
        if typ == pyb.GEOM_BOX:
            return Collision(pos, orient, typ, extents=dims)
        elif typ == pyb.GEOM_SPHERE:
            return Collision(pos, orient, typ, radius=dims[0])
        elif typ == pyb.GEOM_MESH:
            return Collision(pos, orient, typ, mesh_filename=filename,
                             mesh_scale=dims)
        elif typ == pyb.GEOM_CYLINDER or typ == pyb.GEOM_CAPSULE:
            return Collision(pos, orient, typ, radius=dims[1], length=dims[0])
        else:
            raise NotImplementedError("Unknown collision shape: '{!s}'"
                                      .format(typ))


class Visual(object):
    """PyBullet visual object"""
    def __init__(self, collision, rgba):
        self.collision = collision
        self.rgba = rgba

    def __repr__(self):
        return "Visual({!s})".format(GEOM_NAME[self.collision.type])

    @staticmethod
    def from_visual(visual):
        """Create a Visual object from a PyBullet VisualShapeData object"""
        coll = Collision.from_shape(visual)
        rgba = visual[7]
        return Visual(coll, rgba)


class Inertial(object):
    """Inertial abstraction object"""
    def __init__(self, mass, inertia, origin, orientation):
        self.mass = mass
        self.inertia = inertia
        self.origin = np.array(origin)
        self.orientation = np.array(orientation)

    @staticmethod
    def from_dynamics(dynamics):
        """Create Inertial from a PyBullet dynamics info object"""
        return Inertial(dynamics[0], dynamics[1], dynamics[3], dynamics[4])


class Link(object):
    """A link element of a body"""
    def __init__(self, inertia, visuals, collisions):
        self.inertial = inertia
        self.visuals = visuals
        self.collisions = collisions
        self.position = np.zeros(3)
        self.orientation = np.array([0., 0., 0., 1.])

    def __repr__(self):
        return "Link(visual: {!s}, collision: {!s})".format(
            self.visuals, self.collisions)

    @staticmethod
    def from_id(uid, lid, client):
        """Create a link object from unique ID and link ID"""
        global LINK_CACHE
        name = client.getBodyInfo(uid)[lid + 1]
        if name not in LINK_CACHE:
            inertia = Inertial.from_dynamics(client.getDynamicsInfo(uid, lid))
            visuals = [Visual.from_visual(v)
                       for v in client.getVisualShapeData(uid)
                       # Need to explicitly filter out visuals
                       if v[1] == lid]
            assert visuals, "No visuals created for link: {:d}".format(uid)
            # No filtering is needed for collisions since the method accepts
            # link ID
            collisions = [Collision.from_shape(s)
                          for s in client.getCollisionShapeData(uid, lid)]
            assert collisions, "No collisions created for link ({:d}: {:d})".format(
                uid, lid)
            LINK_CACHE[name] = Link(inertia, visuals, collisions)
        return copy.deepcopy(LINK_CACHE[name])


class Joint(object):
    """A joint connecting two 'Link' objects"""
    def __init__(self):
        self.parent = -1
        self.child = -1
        self.type = pyb.JOINT_FIXED
        self.axis = (1., 0., 0.)

    def __repr__(self):
        name_mapping = {pyb.JOINT_FIXED: "Fixed",
                        pyb.JOINT_REVOLUTE: "Revolute"}
        return "{!s}(parent: {:d}, child: {:d})".format(
            name_mapping[self.type], self.parent, self.child)

    @staticmethod
    def from_id(uid, jid, client):
        """Create a joint object from a unique ID and joint ID"""
        result = Joint()
        j_info = client.getJointInfo(uid, jid)
        result.type = j_info[2]
        result.axis = j_info[13]
        return result


class MultiBodyBuilder(object):
    """MultiBody builder interface"""
    def __init__(self, morphology, id_mapping, client, flags=0):
        self.client = client
        self.flags = flags
        self._links = []
        self._joints = []
        self._scan(morphology, id_mapping)

    def __len__(self):
        return len(self._links)

    def _scan(self, morphology, ids):
        """Instantiate a multibody from the given morphology and ID mapping"""
        bid_mapping = {}
        # Iterate morphology to convert all nodes to internal representation
        for module in morphology:
            # ID of module as seen in the simulation
            m_id = ids[module]
            # Convert module to multibody link(s)
            links, joints = self._convert_body(m_id)
            # NOTE: Special handling for modules
            if isinstance(module, Servo):
                # If the module is a servo we need to reverse add joint and
                # links for connecting to other modules
                links[0], links[1] = links[1], links[0]
                links[1].position = links[0].position * -1.
                links[1].orientation = links[0].orientation
            # Iterate joints and connect IDs
            for i, joint in enumerate(joints):
                # NOTE: 'len(self)' will be the ID of the first link added
                # through 'extend' below, all links will then follow this set
                # pattern
                joint.parent = i + len(self)
                joint.child = i + 1 + len(self)
            # Connect joints with link IDs
            if module.parent:
                # Since this module has a parent we add a fixed joint
                joints.insert(0, Joint())
                # Create mapping between the first link in the body and
                # desired parent
                joints[0].parent = bid_mapping[module.parent]
                joints[0].child = len(self)
                links[0].position = module.connection[0] * 2.
                links[0].orientation = (module.parent.orientation.T + module.orientation).as_quat()
            else:
                # Set root element position equal to module position
                links[0].position = module.position
                links[0].orientation = module.orientation.as_quat()
            # Add current module to mapping
            bid_mapping[module] = len(self) + len(links) - 1
            # Lastly add elements to internal state for later
            self._links.extend(links)
            self._joints.extend(joints)

    def _convert_body(self, uid):
        """Convert the body with the given unique ID into links and joints"""
        links = []
        joints = []
        # The will always be a root element present
        links.append(Link.from_id(uid, -1, self.client))
        # If the body contains joints we need to iterate the joints and add
        # links
        for i in range(self.client.getNumJoints(uid)):
            links.append(Link.from_id(uid, i, self.client))
            joints.append(Joint.from_id(uid, i, self.client))
            # Update link with position info relative to parent
            j_info = self.client.getJointInfo(uid, i)
            links[-1].position = np.array(j_info[14])
            links[-1].orientation = j_info[15]
        return links, joints

    def build(self, client=None):
        """Build the MultiBody in the client given, if no client is supplied
        the initial client will be used"""
        assert len(self._links) - 1 == len(self._joints)
        client = client if client else self.client
        # Extract information about base
        base = self._links[0]
        base_coll, base_vis = MultiBodyBuilder.create_shapes(base, client)
        # Iterate links and joints for instancing
        masses = []
        coll_indices = []
        vis_indices = []
        positions = []
        orientations = []
        inertial_pos = []
        inertial_orient = []
        parent_indices = []
        joint_types = []
        joint_axis = []
        for link, joint in zip(self._links[1:], self._joints):
            masses.append(link.inertial.mass)
            c_id, v_id = MultiBodyBuilder.create_shapes(link, client)
            coll_indices.append(c_id)
            vis_indices.append(v_id)
            positions.append(link.position)
            orientations.append(link.orientation)
            inertial_pos.append(link.inertial.origin)
            inertial_orient.append(link.inertial.orientation)
            parent_indices.append(joint.parent)
            joint_types.append(joint.type)
            joint_axis.append(joint.axis)
        # Create multibody
        uid = client.createMultiBody(
            # Base definition
            baseMass=base.inertial.mass,
            baseCollisionShapeIndex=base_coll,
            baseVisualShapeIndex=base_vis,
            basePosition=base.position,
            baseOrientation=base.orientation,
            baseInertialFramePosition=base.inertial.origin,
            baseInertialFrameOrientation=base.inertial.orientation,
            # Link definitions
            linkMasses=masses,
            linkCollisionShapeIndices=coll_indices,
            linkVisualShapeIndices=vis_indices,
            linkPositions=positions,
            linkOrientations=orientations,
            linkInertialFramePositions=inertial_pos,
            linkInertialFrameOrientations=inertial_orient,
            linkParentIndices=parent_indices,
            linkJointTypes=joint_types,
            linkJointAxis=joint_axis,
            # Additional parameters
            flags=self.flags)
        return uid

    @staticmethod
    def create_shapes(link, client):
        """Create visual and collision shapes in the given client"""
        coll_index = client.createCollisionShapeArray(
            shapeTypes=[c.type for c in link.collisions],
            radii=[c.radius for c in link.collisions],
            halfExtents=[c.extents for c in link.collisions],
            lengths=[c.length for c in link.collisions],
            fileNames=[c.mesh_filename for c in link.collisions],
            meshScales=[c.mesh_scale for c in link.collisions],
            collisionFramePositions=[c.position for c in link.collisions],
            collisionFrameOrientations=[c.orientation
                                        for c in link.collisions])
        vis_index = client.createVisualShapeArray(
            shapeTypes=[c.collision.type for c in link.visuals],
            radii=[c.collision.radius for c in link.visuals],
            halfExtents=[c.collision.extents for c in link.visuals],
            lengths=[c.collision.length for c in link.visuals],
            fileNames=[c.collision.mesh_filename for c in link.visuals],
            meshScales=[c.collision.mesh_scale for c in link.visuals],
            rgbaColors=[c.rgba for c in link.visuals],
            visualFramePositions=[c.collision.position for c in link.visuals],
            visualFrameOrientations=[c.collision.orientation
                                     for c in link.visuals])
        return coll_index, vis_index
