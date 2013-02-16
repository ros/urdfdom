import string
import yaml, collections
import rospy
from lxml import etree
from xml.etree.ElementTree import ElementTree # Different implementations mix well
from cStringIO import StringIO

verbose = True

def dict_sub(obj, keys):
	# Could use lambdas and maps, but we'll do straightforward stuffs
	sub = {}
	for key in keys:
		sub[key] = obj[key]
	return sub

def pfloat(x):
	return "{0}".format(x).rstrip('.')

def to_xml_list(values):
	return ' '.join(values)

def to_xml_vector(xyz):
	return to_xml_list(map(pfloat, xyz))

def to_xml_str(value):
	if type(value) in [list, tuple]:
		if value and type(list[0]) in [int, float]:
			return to_xml_vector(value)
		else:
			return to_xml_list(value)
	elif type(value) == float:
		return pfloat(value)
	elif type(value) != str:
		return str(value)
	else:
		return value

def from_xml_list(s):
	return s.split()

def from_xml_vector(s):
	return map(float, from_xml_list(s))


def node_sub(doc, sub):
	if sub is None:
		return None
	if type(sub) == str:
		return etree.SubElement(doc, sub)
	elif isinstance(sub, HybridObject):
		return sub.to_xml(doc)
	else:
		raise Exception('Invalid sub value')

def node_sub_pairs(node, pairs):
	""" Multiple sub elements with text only """
	for (key, value) in pairs:
		if isinstance(value, HybridObject):
			value.to_xml(node)
		elif value is not None:
			subby(node, key).text = to_xml_str(value)

def node_sub_dict(node, obj):
	sub_value_pairs(node, obj.iteritems())

def node_set(node, name, value):
	""" Set attribute """
	if value is not None:
		node.set(name, to_xml_str(value))
	
def node_set_pairs(node, pairs):
	""" Multi attributes in a list of pairs """
	for (key, value) in pairs:
		if value is not None:
			node.set(key, to_xml_str(value))

def node_set_dict(node, obj):
	node_set_pairs(node, obj) 

def node_get(node, name, conv = float):
	value = node.get(name)
	if value is None:
		return None
	else:
		return conv(value)

def to_yaml(obj):
	""" Simplify yaml representation for pretty printing """
	# Is there a better way to do this by adding a representation with yaml.Dumper?
	# Ordered dict: http://pyyaml.org/ticket/29#comment:11
	if obj is None or type(obj) in [str, unicode]:
		out = str(obj)
	elif type(obj) in [int, float]:
		return obj
	elif isinstance(obj, HybridObject):
		out = obj.to_yaml()
	elif type(obj) == dict:
		out = {}
		for (var, value) in obj.iteritems():
			out[str(var)] = to_yaml(value)
	elif isinstance(obj, collections.Iterable):
		out = [to_yaml(item) for item in obj]
	else:
		out = str(obj)
	return out

# Reflect basic types?
# Register variable name types, or just use some basic stuff to keep it short and simple?

class HybridObject(object):
	""" Raw python object for yaml / xml representation """
	XML_TAG = None
	XML_SUB = node_sub_pairs
	
	def get_var_pairs(self, var_list_var = 'var_list'):
		all = vars(self)
		var_list = getattr(self, var_list_var, None)
		if var_list is not None:
			out = [[var, all[var]] for key in var_list]
		else:
			out = all.items() 
		return out
	
	def to_xml(self, doc):
		""" For URDF, will assume that this shall be a set of attributes """
		papa = self.__class__
		assert papa.tag is not None
		node = node_sub(doc, papa.tag)
		papa.XML_SUB(node, self.get_var_pairs())
		return node
	
	def to_yaml(self):
		return to_yaml(self.get_var_pairs())
		
	def __str__(self):
		return yaml.dump(self.to_yaml()).rstrip() # Good idea? Will it remove other important things?


class Transmission(HybridObject):
	XML_TAG = 'transmission'
	
	def __init__(self, name = None, joint = None, actuator = None, mechanicalReduction = 1):
		if name and not joint:
			joint = name
		if joint and not actuator:
			actuator = '{}_motor'.format(joint)
		self.name = name
		self.joint = joint
		self.actuator = actuator
		self.mechanicalReduction = mechanicalReduction
	
	@staticmethod
	def from_xml(node):
		name = node.get('name')
		joint = None
		actuator = None
		mechanicalReduction = None
		for child in node.getchildren():
			if child.tag == 'joint':
				joint = child.get('name')
			elif child.tag == 'actuator':
				actuator = child.get('name')
			elif child.tag == 'mechanicalReduction':
				mechanicalReduction = float(child.text)
		return Transmission(name, actuator, joint, mechanicalReduction)
	
	def to_xml(doc):
		node = sub_node(doc, self.__class__.XML_TAG)
		node.set('name') = self.name
		sub_node(node, 'joint').set('name') = self.joint
		sub_node(node, 'actuator').set('name') = self.actuator
		sub_node(node, 'mechanicalReduction').text = to_xml_str(self.mechanicalReduction)

class Collision(HybridObject):
	XML_TAG = 'collision'
	
	def __init__(self, geometry = None, origin = None):
		self.geometry = geometry
		self.origin = origin

	@staticmethod
	def from_xml(node):
		geometry = None
		origin = None
		for child in node.getchildren():
			if child.tag == 'geometry':
				geometry = Geometry.from_xml(child)
			elif child.tag == 'origin':
				origin = Pose.from_xml(child)
			else:
				if verbose:
					rospy.logwarn("Unknown collision element '%s'"%child.tag)
		return Collision(geometry, origin)

class Color(HybridObject):
	XML_TAG = 'color'
	XML_SUB = node_set_pairs
	
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
			self.rgba += [0.]
		if self.rgba is not None and len(self.rgba) != 4:
			raise Exception('Invalid color argument count')

	@staticmethod
	def from_xml(node):
		rgba = node.get('rgba')
		if rgba is not None:
			rgba = from_xml_vector(rgba)
		return Color(rgba)

class Dynamics(HybridObject):
	XML_SUB = node_set_pairs
	def __init__(self, damping=None, friction=None):
		self.damping = damping
		self.friction = friction

	@staticmethod
	def from_xml(node):
		damping = node_get(node, 'damping')
		friction = node_get(node, 'friction')
		return Dynamics(damping, friction)


class Geometry(HybridObject):
	XML_SUB = node_set_pairs
	
	def __init__(self):
		pass

	@staticmethod
	def from_xml(node):
		shape = node.getchildren()[0]
		if shape.tag == 'box':
			return Box.from_xml(shape)
		elif shape.tag == 'cylinder':
			return Cylinder.from_xml(shape)
		elif shape.tag == 'sphere':
			return Sphere.from_xml(shape)
		elif shape.tag == 'mesh':
			return Mesh.from_xml(shape)
		else:
			if verbose:
				rospy.logwarn("Unknown shape %s"%child.tag)
	
	def to_xml(self, doc, withGeometry = True):
		if withGeometry:
			geom = node_sub(doc, 'geometry')
			parent = geom
		else:
			parent = doc
		return self.to_xml_geom(self, parent)
	
	def to_xml_geom(self, doc):
		return HybridObject.to_xml(self, doc)

class Box(Geometry):
	XML_TAG = 'box'
	def __init__(self, size = None):
		if size is None:
			self.size = None
		elif len(size) == 3:
			self.size = size
		else:
			raise Exception('Invalid size')

	@staticmethod
	def from_xml(node):
		size = node_get(node, 'size', from_xml_vector)
		return Box(size)

class Cylinder(Geometry):
	XML_TAG = 'cylinder'
	def __init__(self, radius = 0.0, length = 0.0):
		self.radius = radius
		self.length = length

	@staticmethod
	def from_xml(node):
		radius = node_get(node, 'radius')
		length = node_get(node, 'length')
		return Cylinder(radius, length)

class Sphere(Geometry):
	XML_TAG = 'sphere'
	
	def __init__(self, radius=0.0):
		self.radius = radius

	@staticmethod
	def from_xml(node):
		radius = node_get(node, 'radius')
		return Sphere(radius)


class Mesh(Geometry):
	XML_TAG = 'mesh'
	
	def __init__(self, filename=None, scale=None):
		self.filename = filename
		self.scale = scale

	@staticmethod
	def from_xml(node):
		filename = node.get('filename')
		scale = node_get(node, 'scale', from_xml_vector)
		return Mesh(filename, scale)

class Inertia(HybridObject):
	XML_TAG = 'inertia'
	KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
	
	def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
		self.ixx = ixx
		self.ixy = ixy
		self.ixz = ixz
		self.iyy = iyy
		self.iyz = iyz
		self.izz = izz
		
	@staticmethod
	def from_xml(node):
		inertia = {}
		for v in Inertial.KEYS:
			inertia[v] = float(child.get(v))
		return Inertia(**inertia)

class Inertial(HybridObject):
	XML_TAG = 'inertial'
	
	def __init__(self, mass=0.0, inertia = None, origin=None):
		self.mass = mass
		self.inertia = inertia
		self.origin = origin

	@staticmethod
	def from_xml(node):
		inertia = None
		mass = 0.0
		origin = None
		for child in node.getchildren():
			if child.tag == 'inertia':
				inertia = Inertia.from_xml(child)
			elif child.tag == 'mass':
				mass = float(child.get('value'))
			elif child.tag == 'origin':
				origin = Pose.from_xml(child)
		
		return Inertial(mass, **inertia,origin)

	def to_xml(self, doc):
		node = node_sub(doc, self.__class__.XML_TAG)
		node_sub(node, 'mass').set('value', to_xml_str(self.mass))
		node_sub(node, self.inertia)
		node_sub(node, self.origin)
		return node

class Joint(HybridObject):
	XML_TAG = 'joint'
	types = ['unknown', 'revolute', 'continuous', 'prismatic', 'floating', 'planar', 'fixed']

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
		for child in node.getchildren():
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

	def to_xml(self, doc):
		node = node_sub(doc, self.__class__.XML_TAG)
		node_set(node, 'name', self.name)
		if verbose and self.type not in Joint.TYPES:
			rospy.logwarn("Unknown joint type '%s'" % self.type)
		set_attribute(xml, "type", self.type)
		node_sub(node, 'parent').set('link', self.parent)
		node_sub(node, 'child').set('link', self.child)
		node_sub(node, self.origin)
		if self.axis is not None:
			xml.appendChild( short(doc, "axis", "xyz", self.axis) )
		add(doc, xml, self.limits)
		add(doc, xml, self.dynamics)
		add(doc, xml, self.safety)
		add(doc, xml, self.calibration)
		return xml


#FIXME: we are missing the reference position here.
class JointCalibration(HybridObject):
	def __init__(self, rising=None, falling=None):
		self.rising = rising
		self.falling = falling

	@staticmethod
	def from_xml(node):
		jc = JointCalibration()
		if node.hasAttribute('rising'):
			jc.rising = float( node.get('rising') )
		if node.hasAttribute('falling'):
			jc.falling = float( node.get('falling') )
		return jc

	def to_xml(self, doc):
		xml = doc.createElement('calibration')
		set_attribute(xml, 'rising', self.rising)
		set_attribute(xml, 'falling', self.falling)
		return xml

class JointLimit(HybridObject):
	def __init__(self, effort, velocity, lower=None, upper=None):
		self.effort = effort
		self.velocity = velocity
		self.lower = lower
		self.upper = upper

	@staticmethod
	def from_xml(node):
		jl = JointLimit( float( node.get('effort') ) ,
						 float( node.get('velocity')))
		if node.hasAttribute('lower'):
			jl.lower = float( node.get('lower') )
		if node.hasAttribute('upper'):
			jl.upper = float( node.get('upper') )
		return jl

	def to_xml(self, doc):
		xml = doc.createElement('limit')
		set_attribute(xml, 'effort', self.effort)
		set_attribute(xml, 'velocity', self.velocity)
		set_attribute(xml, 'lower', self.lower)
		set_attribute(xml, 'upper', self.upper)
		return xml

#FIXME: we are missing __str__ here.
class JointMimic(HybridObject):
	def __init__(self, joint_name, multiplier=None, offset=None):
		self.joint_name = joint_name
		self.multiplier = multiplier
		self.offset = offset

	@staticmethod
	def from_xml(node):
		mimic = JointMimic( node.get('joint') )
		if node.hasAttribute('multiplier'):
			mimic.multiplier = float( node.get('multiplier') )
		if node.hasAttribute('offset'):
			mimic.offset = float( node.get('offset') )
		return mimic

	def to_xml(self, doc):
		xml = doc.createElement('mimic')
		set_attribute(xml, 'joint', self.joint_name)
		set_attribute(xml, 'multiplier', self.multiplier)
		set_attribute(xml, 'offset', self.offset)
		return xml

class Link(HybridObject):
	def __init__(self, name, visual=None, inertial=None, collision=None):
		self.name = name
		self.visual = visual
		self.inertial=inertial
		self.collision=collision

	@staticmethod
	def from_xml(node):
		link = Link(node.get('name'))
		for child in node.getchildren():
			if child.tag == 'visual':
				link.visual = Visual.from_xml(child)
			elif child.tag == 'collision':
				link.collision = Collision.from_xml(child)
			elif child.tag == 'inertial':
				link.inertial = Inertial.from_xml(child)
			else:
				if verbose:
					rospy.logwarn("Unknown link element '%s'"%child.tag)
		return link

	def to_xml(self, doc):
		xml = doc.createElement("link")
		xml.setAttribute("name", self.name)
		add( doc, xml, self.visual)
		add( doc, xml, self.collision)
		add( doc, xml, self.inertial)
		return xml

class Material(HybridObject):
	def __init__(self, name=None, color=None, texture=None):
		self.name = name
		self.color = color
		self.texture = texture

	@staticmethod
	def from_xml(node):
		material = Material()
		if node.hasAttribute('name'):
			material.name = node.get('name')
		for child in node.getchildren():
			if child.tag == 'color':
				material.color = Color.from_xml(child)
			elif child.tag == 'texture':
				material.texture = child.get('filename')
			else:
				if verbose:
					rospy.logwarn("Unknown material element '%s'"%child.tag)

		return material

	def to_xml(self, doc):
		xml = doc.createElement("material")
		set_attribute(xml, "name", self.name)
		add( doc, xml, self.color )

		if self.texture is not None:
			text = doc.createElement("texture")
			text.setAttribute('filename', self.texture)
			xml.appendChild(text)
		return xml


class Pose(HybridObject):
	XML_TAG = 'origin'
	
	def __init__(self, xyz = None, rpy =None):
		self.xyz = xyz
		self.rpy = rpy

	@staticmethod
	def from_xml(node):
		xyz = node_get(node, 'xyz', from_xml_vector)
		rpy = node_get(node, 'rpy', from_xml_vector)
		return Pose(xyz, rpy)


class SafetyController(HybridObject):
	XML_TAG = 'safety_controller'
	XML_SUB = node_set_pairs
	
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

class Visual(HybridObject):
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
		for child in node.getchildren():
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

class XmlTypeTuple:
	def __init__(self, curType, items = {}):
		self.type = curType
		self.items = items

class URDF(HybridObject):
	def __init__(self, name=""):
		self.name = name
		self.elements = []
		self.links = {}
		self.joints = {}
		self.materials = {}
		
		self.types = [Joint, Link, Material, Transmission] # Gazebo
		
		self.tuples = {}
		for curType in self.types:
			self.tuples[curType.XML_TAG] = XmlTypeTuple(curType)
		
		# Other things we don't really care about
		self.joints = self.tuples['joints'].items
		self.links = self.tuples['link'].items

		self.parent_map = {}
		self.child_map = {}

	@staticmethod
	def from_xml_string(xml_string):
		"""Parse a string to create a URDF robot structure."""
		urdf = URDF()
		base = xml.dom.minidom.parseString(xml_string)
		robot = children(base)[0]
		urdf.name = robot.get('name')

		for node in robot.getchildren():
			typeName = node.tag
			curType = self.typeMap.get(typeName, None)
			if curType is None:
				if verbose:
					rospy.logwarn("Unknown robot element '%s'"%node.tag)
			else:
				urdf.add_type(typeName, curType)
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
		curTuple = self.tuples(typeName)
		self.elements.append(elem)
		curTuple.items[elem.name] = elem
		
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
		root.setAttribute("name", self.name)

		for element in self.elements:
			element.to_xml(doc)
			
		# Little hackish at the end
		buffer = StringIO()
		doc = ElementTree(root)
		doc.write(buffer, xml_declaration = True, encoding = 'utf-8')
		return buffer.getvalue()