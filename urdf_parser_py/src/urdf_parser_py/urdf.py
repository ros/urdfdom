import rospy
from urdf_parser_py.basics import *
from urdf_parser_py.reflection import *

# TODO Add in check to see if there are duplicate instances of things that should be unique?
# i.e. Two origins? 

verbose = True

class Pose(XmlObject):
	def __init__(self, xyz=None, rpy=None):
		self.xyz = xyz
		self.rpy = rpy

Pose.XML_REFL = XmlReflection([
	XmlAttribute('xyz', 'vector3'),
	XmlAttribute('rpy', 'vector3')
	])

# Common stuff
nameAttribute = XmlAttribute('name', str)
originElement = XmlElement('origin', Pose, False)

class Transmission(XmlObject):
	def __init__(self, name = None, joint = None, actuator = None, mechanicalReduction = 1):
		self.name = name
		self.joint = joint
		self.actuator = actuator
		self.mechanicalReduction = mechanicalReduction

Transmission.XML_REFL = XmlReflection(params = [
	nameAttribute,
	XmlElement('joint', 'element_name'),
	XmlElement('actuator', 'element_name'),
	XmlElement('mechanicalReduction', float)
	])


class Collision(XmlObject):
	def __init__(self, geometry = None, origin = None):
		self.geometry = geometry
		self.origin = origin

Collision.XML_REFL = XmlReflection([
	originElement,
	XmlElement('geometry', 'geometric')
	])


class Color(UrdfObject):
	def __init__(self, *args):
		# What about named colors?
		count = len(args)
		if count == 4 or count == 3:
			self.rgba = args
		elif count == 1:
			self.rgba = args[0]
		elif count == 0:
			self.rgba = None
		if len(self.rgba) == 3:
			self.rgba += [1.]
		if self.rgba is not None and len(self.rgba) != 4:
			raise Exception('Invalid color argument count')

Color.XML_REFL = XmlReflection([
	XmlAttribute('rgba', 'vector4')
	])


class Dynamics(UrdfObject):
	def __init__(self, damping=None, friction=None):
		self.damping = damping
		self.friction = friction

Dynamics.XML_REFL = XmlReflection([
	XmlAttribute('damping', float, False),
	XmlAttribute('friction', float, False)
	])


class XmlGeometricType(XmlValueType):
	def __init__(self):
		self.dynamic = XmlDynamicType('geometric', {
			'box': Box,
			'cylinder': Cylinder,
			'sphere': Sphere,
			'mesh': Mesh
			})
	
	def from_xml(self, node):
		children = node.getchildren()
		assert len(children) == 1, 'One element only for geometric'
		return self.dynamic.from_xml(children[0])
	
	def to_xml(self, node, obj):
		name = self.dynamic.get_name(obj)
		child = node_add(name)
		obj.to_xml(child)


class Box(XmlObject):
	def __init__(self, size = None):
		self.size = size

Box.XML_REFL = XmlReflection([
	XmlAttribute('size', 'vector3')
	])


class Cylinder(XmlObject):
	def __init__(self, radius = 0.0, length = 0.0):
		self.radius = radius
		self.length = length

Cylinder.XML_REFL = XmlReflection([
	XmlAttribute('radius', float),
	XmlAttribute('length', float)
	])


class Sphere(XmlObject):
	def __init__(self, radius=0.0):
		self.radius = radius

Sphere.XML_REFL = XmlReflection([
	XmlAttribute('radius', float)
	])


class Mesh(XmlObject):
	def __init__(self, filename = None, scale = None):
		self.filename = filename
		self.scale = scale

Mesh.XML_REFL = XmlReflection([
	XmlAttribute('filename', str),
	XmlAttribute('scale', 'vector3')
	])


class Inertia(XmlObject):
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

Inertia.XML_REFL = XmlReflection([XmlAttribute(key, float) for key in Inertia.KEYS])


class Inertial(XmlObject):
	def __init__(self, mass = 0.0, inertia = None, origin=None):
		self.mass = mass
		self.inertia = inertia
		self.origin = origin

Inertial.XML_REFL = XmlReflection([
	XmlElement('mass', 'simple_value'),
	XmlElement('inertia', Inertia, False),
	originElement
	])


class Joint(UrdfObject):
	XML_TAG = 'joint'
	TYPES = ['unknown', 'revolute', 'continuous', 'prismatic', 'floating', 'planar', 'fixed']

	def __init__(self, name, parent, child, joint_type, axis=None, origin=None,
				 limits=None, dynamics=None, safety=None, calibration=None,
				 mimic=None):
		self.name = name
		self.parent = parent
		self.child = child
		self.type = joint_type
		self.axis = axis
		self.origin = origin
		self.limits = limits
		self.dynamics = dynamics
		self.safety = safety
		self.calibration = calibration
		self.mimic = mimic

	@staticmethod
	def from_xml(node):
		joint = Joint(node.get('name'), None, None,
					  node.get('type'))
		for child in children(node):
			if child.tag == 'parent':
				joint.parent = child.get('link')
			elif child.tag == 'child':
				joint.child = child.get('link')
			elif child.tag == 'axis':
				joint.axis = node_get(child, 'xyz', from_xml_vector)
			elif child.tag == 'origin':
				joint.origin = Pose.from_xml(child)
			elif child.tag == 'limit':
				joint.limits = JointLimit.from_xml(child)
			elif child.tag == 'dynamics':
				joint.dynamics = Dynamics.from_xml(child)
			elif child.tag == 'safety_controller':
				joint.safety = SafetyController.from_xml(child)
			elif child.tag == 'calibration':
				joint.calibration = JointCalibration.from_xml(child)
			elif child.tag == 'mimic':
				joint.mimic = JointMimic.from_xml(child)
			else:
				if verbose:
					rospy.logwarn("Unknown joint element '%s'"%child.tag)
		return joint

add_xml_value_type('element_link', XmlSimpleElementType('link', str))

Joint.XML_REFL = XmlReflection([
	nameAttribute,
	XmlAttribute('type', str),
	XmlElement('parent', 'element_link'),
	XmlElement('child', 'element_link'),
	XmlElement('axis', Axis),
	XmlElement('limit', JointLimit, False),
	XmlElement('dynamics', Dynamics, False),
	XmlElement('safety_controller', SafetyController, False),
	XmlElement('calibration', JointCalibration, False),
	XmlElement('mimic', JointMimic, False)
	])


#FIXME: we are missing the reference position here.
class JointCalibration(XmlObject):
	def __init__(self, rising=None, falling=None):
		self.rising = rising
		self.falling = falling

JointCalibration.XML_REFL = XmlReflection([
	XmlAttribute('rising', float),
	XmlAttribute('falling', float)
	])

class JointLimit(UrdfObject):
	def __init__(self, effort=None, velocity=None, lower=None, upper=None):
		self.effort = effort
		self.velocity = velocity
		self.lower = lower
		self.upper = upper

JointLimit.XML_REFL = XmlReflection([
	XmlAttribute('effort', float),
	XmlAttribute('velocity', float),
	XmlAttribute('lower', float),
	XmlAttribute('upper', float)
	])

#FIXME: we are missing __str__ here.
class JointMimic(XmlObject):
	def __init__(self, joint_name, multiplier=None, offset=None):
		self.joint_name = joint_name
		self.multiplier = multiplier
		self.offset = offset

JointMimic.XML_REFL = XmlReflection([
	XmlAttribute('joint', str),
	XmlAttribute('multiplier', float, False),
	XmlAttribute('offset', float, False)
	])


class Link(XmlObject):
	def __init__(self, name, visual=None, inertial=None, collision=None, origin = None):
		self.name = name
		self.visual = visual
		self.inertial=inertial
		self.collision=collision
		self.origin = origin

	@staticmethod
	def from_xml(node):
		link = Link(node.get('name'))
		for child in children(node):
			if child.tag == 'visual':
				link.visual = Visual.from_xml(child)
			elif child.tag == 'collision':
				link.collision = Collision.from_xml(child)
			elif child.tag == 'inertial':
				link.inertial = Inertial.from_xml(child)
			elif child.tag == 'origin':
				link.origin = Pose.from_xml(child)
			else:
				if verbose:
					rospy.logwarn("Unknown link element '%s'"%child.tag)
		return link

class Texture(UrdfObject):
	XML_TAG = 'texture'
	XML_WRITE = 'set'
	
	def __init__(self, filename = None):
		self.filename = filename
		
	@staticmethod
	def from_xml(node):
		filename = node.get('filename')
		return Texture(filename)


class Material(UrdfObject):
	XML_TAG = 'material'
	def __init__(self, name=None, color=None, texture=None):
		self.name = name
		self.color = color
		self.texture = texture

	@staticmethod
	def from_xml(node):
		name = node.get('name')
		color = None
		texture = None
		for child in children(node):
			if child.tag == 'color':
				color = Color.from_xml(child)
			elif child.tag == 'texture':
				texture = Texture.from_xml(child)
			else:
				if verbose:
					rospy.logwarn("Unknown material element '%s'"%child.tag)
		return Material(name, color, texture)


class SafetyController(UrdfObject):
	XML_TAG = 'safety_controller'
	XML_WRITE = 'set'
	
	def __init__(self, velocity, position=None, lower=None, upper=None):
		self.k_velocity = velocity
		self.k_position = position
		self.soft_lower_limit = lower
		self.soft_upper_limit = upper

	@staticmethod
	def from_xml(node):
		velocity = node_get(node, 'k_velocity')
		k_position = node_get(node, 'k_position')
		soft_lower_limit = node_get(node, 'soft_lower_limit')
		soft_upper_limit = node_get(node, 'soft_upper_limit')
		return SafetyController(k_velocity, k_position, soft_lower_limit, soft_upper_limit)

class Visual(UrdfObject):
	XML_TAG = 'visual'
	
	def __init__(self, geometry = None, material = None, origin = None):
		self.geometry = geometry
		self.material = material
		self.origin = origin

	@staticmethod
	def from_xml(node):
		geometry = None
		origin = None
		material = None
		for child in children(node):
			if child.tag == 'geometry':
				geometry = Geometry.from_xml(child)
			elif child.tag == 'origin':
				origin = Pose.from_xml(child)
			elif child.tag == 'material':
				material = Material.from_xml(child)
			else:
				if verbose:
					rospy.logwarn("Unknown visual element '%s'"%child.tag)
		return Visual(geometry, material, origin)

class Gazebo(UrdfObject):
	XML_TAG = 'gazebo'
	
	def __init__(self, xml = None):
		self.xml = xml
		# Hack..
		self.name = ''
	
	@staticmethod
	def from_xml(node):
		return Gazebo(node)
	
	def to_xml(self, node):
		node_add(node, self.xml)

class URDF(UrdfObject):
	XML_FACTORY = None
	
	def __init__(self, name = ''):
		self.name = name
		self.elements = []
		self.links = {}
		self.joints = {}
		self.materials = {}
		
		self.maps = {}
		for name in URDF.XML_FACTORY.typeMap:
			self.maps[name] = {}
		
		# Other things we don't really care about
		self.joints = self.maps['joint']
		self.links = self.maps['link']

		self.parent_map = {}
		self.child_map = {}
		
		self.var_list = ['name', 'links', 'joints']

	@staticmethod
	def from_xml_string(xml_string):
		"""Parse a string to create a URDF robot structure."""
		urdf = URDF()
		robot = etree.fromstring(xml_string)
		urdf.name = robot.get('name')

		for node in children(robot):
			element = URDF.XML_FACTORY.from_xml(node, False)
			if element:
				urdf.add_element(element.XML_TAG, element)
		return urdf

	@staticmethod
	def load_xml_file(filename):
		"""Parse a file to create a URDF robot structure."""
		return URDF.from_xml_string(open(filename, 'r').read())

	@staticmethod
	def load_from_parameter_server(key = 'robot_description'):
		"""
		Retrieve the robot model on the parameter server
		and parse it to create a URDF robot structure.

		Warning: this requires roscore to be running.
		"""
		return URDF.from_xml_string(rospy.get_param(key))
	
	def add_element(self, typeName, elem):
		self.elements.append(elem)
		
		if hasattr(elem, 'name'):
			self.maps[typeName][elem.name] = elem
		
		if typeName == 'joint':
			joint = elem
			self.parent_map[ joint.child ] = (joint.name, joint.parent)
			if joint.parent in self.child_map:
				self.child_map[joint.parent].append( (joint.name, joint.child) )
			else:
				self.child_map[joint.parent] = [ (joint.name, joint.child) ]

	def add_link(self, link):
		self.add_element('link', link)

	def add_joint(self, joint):
		self.add_element('joint', link)

	def get_chain(self, root, tip, joints=True, links=True, fixed=True):
		chain = []
		if links:
			chain.append(tip)
		link = tip
		while link != root:
			(joint, parent) = self.parent_map[link]
			if joints:
				if fixed or self.joints[joint].joint_type != 'fixed':
					chain.append(joint)
			if links:
				chain.append(parent)
			link = parent
		chain.reverse()
		return chain

	def get_root(self):
		root = None
		for link in self.links:
			if link not in self.parent_map:
				assert root is None, "Multiple roots detected, invalid URDF."
				root = link
		assert root is not None, "No roots detected, invalid URDF."
		return root

	def to_xml(self):
		root = etree.Element('robot')
		root.set('name', self.name)

		for element in self.elements:
			element.to_xml(root)
			
		return xml_string(root)

URDF.XML_FACTORY = UrdfFactory(URDF, [Joint, Link, Material, Transmission, Gazebo])