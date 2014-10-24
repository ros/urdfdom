from urdf_parser_py.xml_reflection.basics import *
import urdf_parser_py.xml_reflection as xmlr

# Add a 'namespace' for names so that things don't conflict between URDF and SDF?
# A type registry? How to scope that? Just make a 'global' type pointer?
# Or just qualify names? urdf.geometric, sdf.geometric

xmlr.start_namespace('urdf')

xmlr.add_type('element_link', xmlr.SimpleElementType('link', str))
xmlr.add_type('element_xyz', xmlr.SimpleElementType('xyz', 'vector3'))

verbose = True

class Pose(xmlr.Object):
	def __init__(self, xyz=None, rpy=None):
		self.xyz = xyz
		self.rpy = rpy
	
	def check_valid(self):
		assert self.xyz is not None or self.rpy is not None
	
	# Aliases for backwards compatibility
	@property
	def rotation(self): return self.rpy
	@rotation.setter
	def rotation(self, value): self.rpy = value
	@property
	def position(self): return self.xyz
	@position.setter
	def position(self, value): self.xyz = value

xmlr.reflect(Pose, params = [
	xmlr.Attribute('xyz', 'vector3', False),
	xmlr.Attribute('rpy', 'vector3', False)
	])


# Common stuff
name_attribute = xmlr.Attribute('name', str)
origin_element = xmlr.Element('origin', Pose, False)

class Color(xmlr.Object):
	def __init__(self, *args):
		# What about named colors?
		count = len(args)
		if count == 4 or count == 3:
			self.rgba = args
		elif count == 1:
			self.rgba = args[0]
		elif count == 0:
			self.rgba = None
		if self.rgba is not None:
			if len(self.rgba) == 3:
				self.rgba += [1.]
			if len(self.rgba) != 4:
				raise Exception('Invalid color argument count')

xmlr.reflect(Color, params = [
	xmlr.Attribute('rgba', 'vector4')
	])


class JointDynamics(xmlr.Object):
	def __init__(self, damping=None, friction=None):
		self.damping = damping
		self.friction = friction

xmlr.reflect(JointDynamics, params = [
	xmlr.Attribute('damping', float, False),
	xmlr.Attribute('friction', float, False)
	])


class Box(xmlr.Object):
	def __init__(self, size = None):
		self.size = size

xmlr.reflect(Box, params = [
	xmlr.Attribute('size', 'vector3')
	])


class Cylinder(xmlr.Object):
	def __init__(self, radius = 0.0, length = 0.0):
		self.radius = radius
		self.length = length

xmlr.reflect(Cylinder, params = [
	xmlr.Attribute('radius', float),
	xmlr.Attribute('length', float)
	])


class Sphere(xmlr.Object):
	def __init__(self, radius=0.0):
		self.radius = radius

xmlr.reflect(Sphere, params = [
	xmlr.Attribute('radius', float)
	])


class Mesh(xmlr.Object):
	def __init__(self, filename = None, scale = None):
		self.filename = filename
		self.scale = scale

xmlr.reflect(Mesh, params = [
	xmlr.Attribute('filename', str),
	xmlr.Attribute('scale', 'vector3', required=False)
	])


class GeometricType(xmlr.ValueType):
	def __init__(self):
		self.factory = xmlr.FactoryType('geometric', {
			'box': Box,
			'cylinder': Cylinder,
			'sphere': Sphere,
			'mesh': Mesh
			})
	
	def from_xml(self, node):
		children = xml_children(node)
		assert len(children) == 1, 'One element only for geometric'
		return self.factory.from_xml(children[0])
	
	def write_xml(self, node, obj):
		name = self.factory.get_name(obj)
		child = node_add(node, name)
		obj.write_xml(child)

xmlr.add_type('geometric', GeometricType())

class Collision(xmlr.Object):
	def __init__(self, geometry = None, origin = None):
		self.geometry = geometry
		self.origin = origin

xmlr.reflect(Collision, params = [
	origin_element,
	xmlr.Element('geometry', 'geometric')
	])


class Texture(xmlr.Object):
	def __init__(self, filename = None):
		self.filename = filename

xmlr.reflect(Texture, params = [
	xmlr.Attribute('filename', str)
	])


class Material(xmlr.Object):
	def __init__(self, name=None, color=None, texture=None):
		self.name = name
		self.color = color
		self.texture = texture
	
	def check_valid(self):
		if self.color is None and self.texture is None:
			xmlr.on_error("Material has neither a color nor texture")

xmlr.reflect(Material, params = [
	name_attribute,
	xmlr.Element('color', Color, False),
	xmlr.Element('texture', Texture, False)
	])


class Visual(xmlr.Object):
	def __init__(self, geometry = None, material = None, origin = None):
		self.geometry = geometry
		self.material = material
		self.origin = origin

xmlr.reflect(Visual, params = [
	origin_element,
	xmlr.Element('geometry', 'geometric'),
	xmlr.Element('material', Material, False)
	])


class Inertia(xmlr.Object):
	KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
	
	def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
		self.ixx = ixx
		self.ixy = ixy
		self.ixz = ixz
		self.iyy = iyy
		self.iyz = iyz
		self.izz = izz
	
	def to_matrix(self):
		return [
			[self.ixx, self.ixy, self.ixz],
			[self.ixy, self.iyy, self.iyz],
			[self.ixz, self.iyz, self.izz]]

xmlr.reflect(Inertia, params = [xmlr.Attribute(key, float) for key in Inertia.KEYS])


class Inertial(xmlr.Object):
	def __init__(self, mass = 0.0, inertia = None, origin=None):
		self.mass = mass
		self.inertia = inertia
		self.origin = origin

xmlr.reflect(Inertial, params = [
	origin_element,
	xmlr.Element('mass', 'element_value'),
	xmlr.Element('inertia', Inertia, False)
	])



#FIXME: we are missing the reference position here.
class JointCalibration(xmlr.Object):
	def __init__(self, rising=None, falling=None):
		self.rising = rising
		self.falling = falling

xmlr.reflect(JointCalibration, params = [
	xmlr.Attribute('rising', float, False),
	xmlr.Attribute('falling', float, False)
	])

class JointLimit(xmlr.Object):
	def __init__(self, effort=None, velocity=None, lower=None, upper=None):
		self.effort = effort
		self.velocity = velocity
		self.lower = lower
		self.upper = upper

xmlr.reflect(JointLimit, params = [
	xmlr.Attribute('effort', float),
	xmlr.Attribute('lower', float, False, 0),
	xmlr.Attribute('upper', float, False, 0),
	xmlr.Attribute('velocity', float)
	])

#FIXME: we are missing __str__ here.
class JointMimic(xmlr.Object):
	def __init__(self, joint_name=None, multiplier=None, offset=None):
		self.joint = joint_name
		self.multiplier = multiplier
		self.offset = offset	

xmlr.reflect(JointMimic, params = [
	xmlr.Attribute('joint', str),
	xmlr.Attribute('multiplier', float, False),
	xmlr.Attribute('offset', float, False)
	])

class SafetyController(xmlr.Object):
	def __init__(self, velocity=None, position=None, lower=None, upper=None):
		self.k_velocity = velocity
		self.k_position = position
		self.soft_lower_limit = lower
		self.soft_upper_limit = upper

xmlr.reflect(SafetyController, params = [
	xmlr.Attribute('k_velocity', float),
	xmlr.Attribute('k_position', float, False, 0),
	xmlr.Attribute('soft_lower_limit', float, False, 0),
	xmlr.Attribute('soft_upper_limit', float, False, 0)
	])

class Joint(xmlr.Object):
	TYPES = ['unknown', 'revolute', 'continuous', 'prismatic', 'floating', 'planar', 'fixed']

	def __init__(self, name=None, parent=None, child=None, joint_type=None,
			axis=None, origin=None,
			limit=None, dynamics=None, safety_controller=None, calibration=None,
			mimic=None):
		self.name = name
		self.parent = parent
		self.child = child
		self.type = joint_type
		self.axis = axis
		self.origin = origin
		self.limit = limit
		self.dynamics = dynamics
		self.safety_controller = safety_controller
		self.calibration = calibration
		self.mimic = mimic
	
	def check_valid(self):
		assert self.type in self.TYPES, "Invalid joint type: {}".format(self.type)
	
	# Aliases
	@property
	def joint_type(self): return self.type
	@joint_type.setter
	def joint_type(self, value): self.type = value

xmlr.reflect(Joint, params = [
	name_attribute,
	xmlr.Attribute('type', str),
	origin_element,
	xmlr.Element('axis', 'element_xyz', False),
	xmlr.Element('parent', 'element_link'),
	xmlr.Element('child', 'element_link'),
	xmlr.Element('limit', JointLimit, False),
	xmlr.Element('dynamics', JointDynamics, False),
	xmlr.Element('safety_controller', SafetyController, False),
	xmlr.Element('calibration', JointCalibration, False),
	xmlr.Element('mimic', JointMimic, False)
	])


class Link(xmlr.Object):
	def __init__(self, name=None, visual=None, inertial=None, collision=None, origin = None):
		self.name = name
		self.visual = visual
		self.inertial = inertial
		self.collision = collision
		self.origin = origin

xmlr.reflect(Link, params = [
	name_attribute,
	origin_element,
	xmlr.Element('inertial', Inertial, False),
	xmlr.Element('visual', Visual, False),
	xmlr.Element('collision', Collision, False)
	])


class Actuator(xmlr.Object):
	def __init__(self, name = None, hardwareInterface = None, mechanicalReduction = 1):
		self.name = name
		self.hardwareInterface = None
		self.mechanicalReduction = None

xmlr.reflect(Actuator, tag = 'actuator', params = [
		name_attribute,
		xmlr.Element('hardwareInterface', str),
		xmlr.Element('mechanicalReduction', float, required = False)
		])

class Transmission(xmlr.Object):
	def __init__(self, name = None, joint = None, actuator = None):
		self.name = name
		self.joint = joint
		self.actuator = actuator

xmlr.reflect(Transmission, tag = 'new_transmission', params = [
	name_attribute,
	xmlr.Attribute('type', str),
	xmlr.Element('joint', 'element_name'),
	xmlr.Element('actuator', Actuator),
	])

class PR2Compensator(xmlr.Object):
	def __init__(self, k_belt=4000.0, kd_motor=10.0, lambda_combined=0.0, lambda_joint=60.0, lambda_motor=60.0, mass_motor=0.05):
		self.k_belt = k_belt
		self.kd_motor = kd_motor
		self.lambda_combined = lambda_combined
		self.lambda_joint = lambda_joint
		self.lambda_motor = lambda_motor
		self.mass_motor = mass_motor

xmlr.reflect(PR2Compensator, tag = 'compensator', params = [
		xmlr.Attribute('k_belt', float),
		xmlr.Attribute('kd_motor', float),
		xmlr.Attribute('lambda_combined', float),
		xmlr.Attribute('lambda_joint', float),
		xmlr.Attribute('lambda_motor', float),
		xmlr.Attribute('mass_motor', float)
		])

class PR2SimulatedActuatedJoint(xmlr.Object):
	def __init__(self, name = None, simulated_reduction = 3000, passive_actuated_joint = None):
		self.name = name
		self.simulated_reduction = simulated_reduction
		self.passive_actuated_joint = passive_actuated_joint

xmlr.reflect(PR2SimulatedActuatedJoint, tag = 'simulated_actuated_joint', params = [
		name_attribute,
		xmlr.Attribute('simulated_reduction', float),
		xmlr.Attribute('passive_actuated_joint', str, False)
		])

class PR2Transmission(xmlr.Object):
	def __init__(self, name = None, joint = None, actuator = None, mechanicalReduction = None, compensator = None, simulated_actuated_joint = None):
		self.name = name
		self.joint = joint
		self.actuator = actuator
		self.mechanicalReduction = mechanicalReduction
		self.compensator = compensator
		self.simulated_actuated_joint = simulated_actuated_joint

xmlr.reflect(PR2Transmission, tag = 'pr2_transmission', params = [
	name_attribute,
	xmlr.Attribute('type', str),
	xmlr.Element('joint', 'element_name'),
	xmlr.Element('actuator', 'element_name'),
	xmlr.Element('mechanicalReduction', float),
	xmlr.Element('compensator', PR2Compensator, False),
	xmlr.Element('simulated_actuated_joint', PR2SimulatedActuatedJoint, False)
	])

class PR2Actuator(xmlr.Object):
	def __init__(self, name = None, mechanicalReduction = 1):
		self.name = name
		self.mechanicalReduction = mechanicalReduction

xmlr.reflect(PR2Actuator, tag = 'pr2_actuator', params = [
	name_attribute,
	xmlr.Attribute('mechanicalReduction', float)
	])

class PR2WristTransmission(xmlr.Object):
	def __init__(self, name = None, type = None, rightActuator = None, leftActuator = None, flexJoint = None, rollJoint = None):
		self.name = name
		self.type = type
                self.rightActuator = rightActuator
                self.leftActuator = leftActuator
                self.flexJoint = flexJoint
                self.rollJoint = rollJoint

xmlr.reflect(PR2WristTransmission, tag = 'pr2_wrist_transmission', params = [
	name_attribute,
	xmlr.Attribute('type', str),
	xmlr.Element('rightActuator', PR2Actuator),
	xmlr.Element('leftActuator', PR2Actuator),
	xmlr.Element('flexJoint', PR2Actuator),
	xmlr.Element('rollJoint', PR2Actuator)
	])


class PR2GapJoint(xmlr.Object):
	def __init__(self, L0=0.0, a=0.0, b=0.0, gear_ratio=40.0, h=0.0, mechanical_reduction=1.0, name=None, phi0=0.5, r=0.0, screw_reduction=0.0, t0=0.0, theta0=0.0):
		self.L0 = L0
		self.a = a
		self.b = b
		self.gear_ratio = gear_ratio
		self.h = h
		self.mechanical_reduction = mechanical_reduction
		self.name = name
		self.phi0 = phi0
		self.r = r
		self.screw_reduction = screw_reduction
		self.t0 = t0
		self.theta0 = theta0

xmlr.reflect(PR2GapJoint, tag = 'pr2_gap_joint', params = [
		xmlr.Attribute('L0', float),
		xmlr.Attribute('a', float),
		xmlr.Attribute('b', float),
		xmlr.Attribute('gear_ratio', float),
		xmlr.Attribute('h', float),
		xmlr.Attribute('mechanical_reduction', float),
		xmlr.Attribute('name', str),
		xmlr.Attribute('phi0', float),
		xmlr.Attribute('r', float),
		xmlr.Attribute('screw_reduction', float),
		xmlr.Attribute('t0', float),
		xmlr.Attribute('theta0', float)
		])

class PR2GripperTransmission(xmlr.Object):
	def __init__(self, name = None, type = None, actuator  = None, gap_joint = None, use_simulated_gripper_joint = None, passive_joint = [], simulated_actuated_joint = None):
		self.aggregate_init()
		self.name = name
		self.type = type
                self.actuator = actuator
                self.gap_joint = gap_joint
                self.use_simulated_gripper_joint = use_simulated_gripper_joint
                self.passive_joint = passive_joint
                self.simulated_actuated_joint = simulated_actuated_joint

xmlr.reflect(PR2GripperTransmission, tag = 'pr2_gripper_transmission', params = [
	name_attribute,
	xmlr.Attribute('type', str),
	xmlr.Element('actuator', 'element_name'),
	xmlr.Element('gap_joint', PR2GapJoint),
	xmlr.Element('use_simulated_gripper_joint', 'element_name'),
	xmlr.AggregateElement('passive_joint', 'element_name', 'passive_joint'),
	xmlr.Element('simulated_actuated_joint', PR2SimulatedActuatedJoint),
	])

#xmlr.add_type('transmission', xmlr.DuckTypedFactory('transmission', [Transmission, PR2WristTransmission, PR2GripperTransmission]))
xmlr.add_type('transmission', xmlr.DuckTypedFactory('transmission', [Transmission, PR2Transmission, PR2WristTransmission, PR2GripperTransmission]))


class Robot(xmlr.Object):
	def __init__(self, name = None):
		self.aggregate_init()
		
		self.name = name
		self.joints = []
		self.links = []
		self.materials = []
		self.gazebos = []
		self.transmissions = []
		
		self.joint_map = {}
		self.link_map = {}

		self.parent_map = {}
		self.child_map = {}
	
	def add_aggregate(self, typeName, elem):
		xmlr.Object.add_aggregate(self, typeName, elem)
		
		if typeName == 'joint':
			joint = elem
			self.joint_map[joint.name] = joint
			self.parent_map[ joint.child ] = (joint.name, joint.parent)
			if joint.parent in self.child_map:
				self.child_map[joint.parent].append( (joint.name, joint.child) )
			else:
				self.child_map[joint.parent] = [ (joint.name, joint.child) ]
		elif typeName == 'link':
			link = elem
			self.link_map[link.name] = link

	def add_link(self, link):
		self.add_aggregate('link', link)

	def add_joint(self, joint):
		self.add_aggregate('joint', joint)

	def get_chain(self, root, tip, joints=True, links=True, fixed=True):
		chain = []
		if links:
			chain.append(tip)
		link = tip
		while link != root:
			(joint, parent) = self.parent_map[link]
			if joints:
				if fixed or self.joint_map[joint].joint_type != 'fixed':
					chain.append(joint)
			if links:
				chain.append(parent)
			link = parent
		chain.reverse()
		return chain

	def get_root(self):
		root = None
		for link in self.link_map:
			if link not in self.parent_map:
				assert root is None, "Multiple roots detected, invalid URDF."
				root = link
		assert root is not None, "No roots detected, invalid URDF."
		return root

	@classmethod
	def from_parameter_server(cls, key = 'robot_description'):
		"""
		Retrieve the robot model on the parameter server
		and parse it to create a URDF robot structure.

		Warning: this requires roscore to be running.
		"""
		# Could move this into xml_reflection
		import rospy
		return cls.from_xml_string(rospy.get_param(key))
	
xmlr.reflect(Robot, tag = 'robot', params = [
# 	name_attribute,
	xmlr.Attribute('name', str, False), # Is 'name' a required attribute?
	xmlr.AggregateElement('link', Link),
	xmlr.AggregateElement('joint', Joint),
	xmlr.AggregateElement('gazebo', xmlr.RawType()),
	xmlr.AggregateElement('transmission', 'transmission'),
	xmlr.AggregateElement('material', Material)
	])

# Make an alias
URDF = Robot

xmlr.end_namespace()
