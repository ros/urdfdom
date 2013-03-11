from urdf_parser_py.basics import *
from urdf_parser_py.reflection import *
from mercurial.hgweb.webcommands import static

# Instead of setting XML_REFL, do something like: xml_reflect(Pose, [...]) ???

verbose = True

class Pose(XmlObject):
	def __init__(self, xyz=None, rpy=None):
		self.xyz = xyz
		self.rpy = rpy
	
	def check_valid(self):
		assert self.xyz is not None or self.rpy is not None

Pose.XML_REFL = XmlReflection([
	XmlAttribute('rpy', 'vector3', False),
	XmlAttribute('xyz', 'vector3', False)
	])


# Common stuff
nameAttribute = XmlAttribute('name', str)
originElement = XmlElement('origin', Pose, False)

class Color(XmlObject):
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

Color.XML_REFL = XmlReflection([
	XmlAttribute('rgba', 'vector4')
	])


class JointDynamics(XmlObject):
	def __init__(self, damping=None, friction=None):
		self.damping = damping
		self.friction = friction

JointDynamics.XML_REFL = XmlReflection([
	XmlAttribute('damping', float, False),
	XmlAttribute('friction', float, False)
	])


class XmlGeometricType(XmlValueType):
	def __init__(self):
		self.factory = XmlFactoryType('geometric', {
			'box': Box,
			'cylinder': Cylinder,
			'sphere': Sphere,
			'mesh': Mesh
			})
	
	def from_xml(self, node):
		children = node.getchildren()
		assert len(children) == 1, 'One element only for geometric'
		return self.factory.from_xml(children[0])
	
	def to_xml(self, node, obj):
		name = self.factory.get_name(obj)
		child = node_add(node, name)
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

add_xml_value_type('geometric', XmlGeometricType())

class Collision(XmlObject):
	def __init__(self, geometry = None, origin = None):
		self.geometry = geometry
		self.origin = origin

Collision.XML_REFL = XmlReflection([
	originElement,
	XmlElement('geometry', 'geometric')
	])


class Texture(XmlObject):
	def __init__(self, filename = None):
		self.filename = filename

Texture.XML_REFL = XmlReflection([
	XmlAttribute('filename', str)
	])


class Material(XmlObject):
	def __init__(self, name=None, color=None, texture=None):
		self.name = name
		self.color = color
		self.texture = texture
	
	def check_valid(self):
		if self.color is None and self.texture is None:
			rospy.logwarn("Material has neither a color nor texture")

Material.XML_REFL = XmlReflection([
	nameAttribute,
	XmlElement('color', Color, False),
	XmlElement('texture', Texture, False)
	])


class Visual(XmlObject):
	def __init__(self, geometry = None, material = None, origin = None):
		self.geometry = geometry
		self.material = material
		self.origin = origin

Visual.XML_REFL = XmlReflection([
	originElement,
	XmlElement('geometry', 'geometric'),
	XmlElement('material', Material, False)
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
	XmlElement('mass', 'element_value'),
	XmlElement('inertia', Inertia, False),
	originElement
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

class JointLimit(XmlObject):
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

class SafetyController(XmlObject):
	def __init__(self, velocity, position=None, lower=None, upper=None):
		self.k_velocity = velocity
		self.k_position = position
		self.soft_lower_limit = lower
		self.soft_upper_limit = upper

SafetyController.XML_REFL = XmlReflection([
	XmlAttribute('k_velocity', float),
	XmlAttribute('k_position', float),
	XmlAttribute('soft_lower_limit', float),
	XmlAttribute('soft_upper_limit', float)
	])

class Joint(XmlObject):
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
		
add_xml_value_type('element_link', XmlSimpleElementType('link', str))
add_xml_value_type('element_xyz', XmlSimpleElementType('xyz', 'vector3'))

Joint.XML_REFL = XmlReflection([
	nameAttribute,
	XmlAttribute('type', str),
	XmlElement('parent', 'element_link'),
	XmlElement('child', 'element_link'),
	originElement,
	XmlElement('axis', 'element_xyz', False),
	XmlElement('limit', JointLimit, False),
	XmlElement('dynamics', JointDynamics, False),
	XmlElement('safety_controller', SafetyController, False),
	XmlElement('calibration', JointCalibration, False),
	XmlElement('mimic', JointMimic, False)
	])





class Link(XmlObject):
	def __init__(self, name=None, visual=None, inertial=None, collision=None, origin = None):
		self.name = name
		self.visual = visual
		self.inertial = inertial
		self.collision = collision
		self.origin = origin

Link.XML_REFL = XmlReflection([
	nameAttribute,
	originElement,
	XmlElement('visual', Visual, False),
	XmlElement('collision', Collision, False),
	XmlElement('inertial', Inertial, False)
	])






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


class Gazebo(XmlObject):
	def __init__(self, xml = None):
		self.xml = xml
	
	def get_refl_vars(self):
		return ['xml']
	
	def load_xml(self, node):
		self.xml = node
	
	def to_xml(self, node):
		# This looks super ugly
		rospy.logwarn('Putting Gazebo tags makes things ugly... Need to fix')
		node.append(self.xml)

# TODO Finish this up by making this use the list element thing like an SDF model
# Rename to 'Robot', so it would be 'urdf.Robot'?
class URDF(XmlObject):
	def __init__(self, name = None):
		self.aggregate_init()
		
		self.name = name
		self.joint = []
		self.link = []
		self.material = []
		self.gazebo = []
		self.transmission = []
		
		self.jointMap = {}
		self.linkMap = {}

		self.parent_map = {}
		self.child_map = {}
	
	def add_aggregate(self, typeName, elem):
		XmlObject.add_aggregate(self, typeName, elem)
		
		if typeName == 'joint':
			joint = elem
			self.jointMap[joint.name] = joint
			self.parent_map[ joint.child ] = (joint.name, joint.parent)
			if joint.parent in self.child_map:
				self.child_map[joint.parent].append( (joint.name, joint.child) )
			else:
				self.child_map[joint.parent] = [ (joint.name, joint.child) ]
		elif typeName == 'link':
			link = elem
			self.linkMap[link.name] = link

	def add_link(self, link):
		self.add_aggregate('link', link)

	def add_joint(self, joint):
		self.add_aggregate('joint', link)

	def get_chain(self, root, tip, joints=True, links=True, fixed=True):
		chain = []
		if links:
			chain.append(tip)
		link = tip
		while link != root:
			(joint, parent) = self.parent_map[link]
			if joints:
				if fixed or self.jointMap[joint].joint_type != 'fixed':
					chain.append(joint)
			if links:
				chain.append(parent)
			link = parent
		chain.reverse()
		return chain

	def get_root(self):
		root = None
		for link in self.linkMap:
			if link not in self.parent_map:
				assert root is None, "Multiple roots detected, invalid URDF."
				root = link
		assert root is not None, "No roots detected, invalid URDF."
		return root


	@staticmethod
	def from_xml_string(xml_string):
		"""Parse a string to create a URDF robot structure."""
		robot = etree.fromstring(xml_string)
		urdf = URDF()
		urdf.load_xml(robot)
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
	
	def to_xml_doc(self):
		robot = etree.Element('robot')
		self.to_xml(robot)
		return robot
	
	def to_xml_string(self):
		return xml_string(self.to_xml_doc())
	
URDF.XML_REFL = XmlReflection([
	nameAttribute,
	XmlAggregateElement('link', Link),
	XmlAggregateElement('joint', Joint),
	XmlAggregateElement('gazebo', Gazebo, isRaw = True),
	XmlAggregateElement('transmission', Transmission),
	XmlAggregateElement('material', Material)
	])