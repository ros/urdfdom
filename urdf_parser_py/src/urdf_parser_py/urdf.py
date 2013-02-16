import string
import yaml, collections
import rospy
from lxml import etree

def dict_sub(obj, keys):
	# Could use lambdas and maps, but we'll do straightforward stuffs
	sub = {}
	for key in keys:
		sub[key] = obj[key]
	return sub

def pfloat(x):
	return "{0}".format(x).rstrip('.')

def to_xml_list(xyz):
	return " ".join(map(pfloat, xyz))

def to_xml_str(value):
	if type(value) in [list, tuple]:
		return to_xml_list(value)
	elif type(value) == float:
		return pfloat(value)
	elif type(value) != str:
		return str(value)
	else:
		return value

def from_xml_list(s):
	return map(float, s.split())

sub_elem = etree.SubElement

def sub_node_pairs(node, pairs):
	""" Multiple sub elements with text only """
	for (key, value) in pairs:
		if isinstance(value, HybridObject):
			value.to_xml(node)
		elif value is not None:
			subby(node, key).text = to_xml_str(value)

def sub_node_dict(node, obj):
	sub_value_pairs(node, obj.iteritems())
	
def sub_attr_pairs(node, pairs):
	""" Multi attributes in a list of pairs """
	for (key, value) in pairs:
		if value is not None:
			node.set(key, to_xml_str(value))

def sub_attr_dict(node, obj):
	sub_attr_pairs(node, obj) 

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

class HybridObject(object):
	""" Raw python object for yaml / xml representation """
	xml_tag = None
	xml_sub = sub_node_dict
	
	def get_var_dict(self):
		out = vars(self)
		var_list = getattr(self, 'var_list', None)
		if var_list is not None:
			out = dict_sub(out, var_list)
		return out
	
	def to_xml(self, doc):
		""" For URDF, will assume that this shall be a set of attributes """
		papa = self.__class__
		assert papa.tag is not None
		node = sub_elem(doc, papa.tag)
		papa.xml_sub(node, self.get_var_dict())
	
	def to_yaml(self):
		return to_yaml(out)
		
	def __str__(self):
		return yaml.dump(self.to_yaml()).rstrip() # Good idea? Will it remove other important things?


class Transmission(HybridObject):
	xml_tag = 'transmission'
	
	def __init__(self, joint = None, mechanicalReduction = 1, actuator = None):
		if joint and not actuator:
			actuator = '{}_motor'.format(joint)
		self.joint = joint
		self.actuator = actuator
		self.mechanicalReduction = mechanicalReduction
	
	@staticmethod
	def from_xml(node):
		joint = None
		actuator = None
		mechanicalReduction = None
		for child in node.getchildren():
			if child.tag == 'joint':
				joint = child.get('name')
			elif child.tag == 'actuator':
				actuator = child.get('name')
			elif:
				mechanicalReduction = child.

class Collision(HybridObject):
	def __init__(self, geometry = None, origin = None):
		self.geometry = geometry
		self.origin = origin

	@staticmethod
	def from_xml(node):
		c = Collision()
		for child in node.getchildren():
			if child.tag == 'geometry':
				c.geometry = Geometry.from_xml(child, verbose)
			elif child.tag == 'origin':
				c.origin = Pose.from_xml(child)
			else:
				if verbose:
					rospy.logwarn("Unknown collision element '%s'"%child.tag)
		return c

	def to_xml(self, doc):
		xml = doc.createElement("collision")
		add(doc, xml, self.geometry)
		add(doc, xml, self.origin)
		return xml

class Color(HybridObject):
	def __init__(self, r=0.0, g=0.0, b=0.0, a=0.0):
		self.rgba=(r,g,b,a)
		self.r = r
		self.g = g
		self.b = b
		self.a = a
		self.yaml_vars = ['r', 'g', 'b', 'a']

	@staticmethod
	def from_xml(node):
		rgba = node.get("rgba").split()
		(r,g,b,a) = [ float(x) for x in rgba ]
		return Color(r,g,b,a)

	def to_xml(self, doc):
		xml = doc.createElement("color")
		set_attribute(xml, "rgba", self.rgba)
		return xml

class Dynamics(HybridObject):
	def __init__(self, damping=None, friction=None):
		self.damping = damping
		self.friction = friction

	@staticmethod
	def from_xml(node):
		d = Dynamics()
		if node.hasAttribute('damping'):
			d.damping = float(node.get('damping'))
		if node.hasAttribute('friction'):
			d.friction = float(node.get('friction'))
		return d

	def to_xml(self, doc):
		xml = doc.createElement('dynamics')
		set_attribute(xml, 'damping', self.damping)
		set_attribute(xml, 'friction', self.friction)
		return xml


class Geometry(HybridObject):
	def __init__(self):
		pass

	@staticmethod
	def from_xml(node):
		shape = node.getchildren()[0]
		if shape.tag=='box':
			return Box.from_xml(shape)
		elif shape.tag=='cylinder':
			return Cylinder.from_xml(shape)
		elif shape.tag=='sphere':
			return Sphere.from_xml(shape)
		elif shape.tag=='mesh':
			return Mesh.from_xml(shape)
		else:
			if verbose:
				rospy.logwarn("Unknown shape %s"%child.tag)
	
	def to_xml(self, doc, withGeometry = True):
		

class Box(Geometry):
	def __init__(self, dims=None):
		if dims is None:
			self.dims = None
		else:
			self.dims = (dims[0], dims[1], dims[2])

	@staticmethod
	def from_xml(node):
		dims = from_xml_list(node.get('size'))
		return Box(dims)

	def to_xml(self, doc):
		xml = doc.createElement("box")
		set_attribute(xml, "size", self.dims)
		geom = doc.createElement('geometry')
		geom.appendChild(xml)
		return geom


class Cylinder(Geometry):
	def __init__(self, radius=0.0, length=0.0):
		self.radius = radius
		self.length = length

	@staticmethod
	def from_xml(node):
		r = node.get('radius')
		l = node.get('length')
		return Cylinder(float(r), float(l))

	def to_xml(self, doc, withGeometry = True):
		xml = doc.createElement("cylinder")
		set_attribute(xml, "radius", self.radius)
		set_attribute(xml, "length", self.length)
		if withGeometry:
			geom = doc.createElement('geometry')
			geom.appendChild(xml)
			return geom
		else:
			return xml

class Sphere(Geometry):
	def __init__(self, radius=0.0):
		self.radius = radius

	@staticmethod
	def from_xml(node):
		r = node.get('radius')
		return Sphere(float(r))

	def to_xml(self, doc):
		xml = doc.createElement("sphere")
		set_attribute(xml, "radius", self.radius)
		geom = doc.createElement('geometry')
		geom.appendChild(xml)
		return geom


class Mesh(Geometry):
	def __init__(self, filename=None, scale=None):
		self.filename = filename
		self.scale = scale

	@staticmethod
	def from_xml(node):
		fn = node.get('filename')
		s = node.get('scale')
		if s == "":
			scale = None
		else:
			scale = from_xml_list(s)
		return Mesh(fn, scale)

	def to_xml(self, doc):
		xml = doc.createElement("mesh")
		set_attribute(xml, "filename", self.filename)
		set_attribute(xml, "scale", self.scale)
		geom = doc.createElement('geometry')
		geom.appendChild(xml)
		return geom


class Inertial(HybridObject):
	def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0,
				 mass=0.0, origin=None):
		self.inertia = {}
		self.inertia['ixx'] = ixx
		self.inertia['ixy'] = ixy
		self.inertia['ixz'] = ixz
		self.inertia['iyy'] = iyy
		self.inertia['iyz'] = iyz
		self.inertia['izz'] = izz
		self.mass = mass
		self.origin = origin

	@staticmethod
	def from_xml(node):
		inert = Inertial()
		for child in node.getchildren():
			if child.tag=='inertia':
				for v in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
					inert.inertia[v] = float(child.get(v))
			elif child.tag=='mass':
				inert.mass = float(child.get('value'))
			elif child.tag == 'origin':
				inert.origin = Pose.from_xml(child)
		return inert

	def to_xml(self, doc):
		xml = doc.createElement("inertial")

		xml.appendChild(short(doc, "mass", "value", self.mass))

		inertia = doc.createElement("inertia")
		for (n,v) in self.inertia.items():
			set_attribute(inertia, n, v)
		xml.appendChild(inertia)

		add(doc, xml, self.origin)
		return xml

class Joint(HybridObject):
	UNKNOWN = 'unknown'
	REVOLUTE = 'revolute'
	CONTINUOUS = 'continuous'
	PRISMATIC = 'prismatic'
	FLOATING = 'floating'
	PLANAR = 'planar'
	FIXED = 'fixed'
	
	TYPES = [UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED]


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
				joint.axis = child.get('xyz')
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
		xml = doc.createElement("joint")
		set_attribute(xml, "name", self.name)
		if verbose and self.type not in Joint.TYPES:
			rospy.logwarn("Unknown joint type '%s'" % self.type)
		set_attribute(xml, "type", self.type)
		xml.appendChild( short(doc, "parent", "link", self.parent) )
		xml.appendChild( short(doc, "child" , "link", self.child ) )
		add(doc, xml, self.origin)
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
				link.visual = Visual.from_xml(child, verbose)
			elif child.tag == 'collision':
				link.collision = Collision.from_xml(child, verbose)
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
	def __init__(self, position=None, rotation=None):
		self.position = position
		self.rotation = rotation

	@staticmethod
	def from_xml(node):
		pose = Pose()
		if node.hasAttribute("xyz"):
			xyz = node.get('xyz').split()
			pose.position = map(float, xyz)
		if node.hasAttribute("rpy"):
			rpy = node.get('rpy').split()
			pose.rotation = map(float, rpy)
		return pose

	def to_xml(self, doc):
		xml = doc.createElement("origin")
		set_attribute(xml, 'xyz', self.position)
		set_attribute(xml, 'rpy', self.rotation)
		return xml


class SafetyController(HybridObject):
	def __init__(self, velocity, position=None, lower=None, upper=None):
		self.velocity = velocity
		self.position = position
		self.lower = lower
		self.upper = upper

	@staticmethod
	def from_xml(node):
		sc = SafetyController( float(node.get('k_velocity')) )
		if node.hasAttribute('soft_lower_limit'):
			sc.lower = float( node.get('soft_lower_limit') )
		if node.hasAttribute('soft_upper_limit'):
			sc.upper = float( node.get('soft_upper_limit') )
		if node.hasAttribute('k_position'):
			sc.position = float( node.get('k_position') )
		return sc

	def to_xml(self, doc):
		xml = doc.createElement('safety_controller')
		set_attribute(xml, 'k_velocity', self.velocity)
		set_attribute(xml, 'k_position', self.position)
		set_attribute(xml, 'soft_upper_limit', self.upper)
		set_attribute(xml, 'soft_lower_limit', self.lower)
		return xml


class Visual(HybridObject):
	def __init__(self, geometry=None, material=None, origin=None):
		self.geometry = geometry
		self.material = material
		self.origin = origin

	@staticmethod
	def from_xml(node):
		v = Visual()
		for child in node.getchildren():
			if child.tag == 'geometry':
				v.geometry = Geometry.from_xml(child, verbose)
			elif child.tag == 'origin':
				v.origin = Pose.from_xml(child)
			elif child.tag == 'material':
				v.material = Material.from_xml(child, verbose)
			else:
				if verbose:
					rospy.logwarn("Unknown visual element '%s'"%child.tag)
		return v

	def to_xml(self, doc):
		xml = doc.createElement("visual")
		add( doc, xml, self.geometry )
		add( doc, xml, self.origin )
		add( doc, xml, self.material )
		return xml

class URDF(HybridObject):
	def __init__(self, name=""):
		self.name = name
		self.elements = []
		self.links = {}
		self.joints = {}
		self.materials = {}

		self.parent_map = {}
		self.child_map = {}
		
		self.yaml_vars = ['name', 'links', 'joints', 'materials']

	@staticmethod
	def parse_xml_string(xml_string):
		"""Parse a string to create a URDF robot structure."""
		urdf = URDF()
		base = xml.dom.minidom.parseString(xml_string)
		robot = children(base)[0]
		urdf.name = robot.get('name')

		for node in children(robot):
			if node.tag == 'joint':
				urdf.add_joint( Joint.from_xml(node, verbose) )
			elif node.tag == 'link':
				urdf.add_link( Link.from_xml(node, verbose) )
			elif node.tag == 'material':
				urdf.elements.append( Material.from_xml(node, verbose) )
			elif node.tag == 'gazebo':
				pass #Gazebo not implemented yet
			elif node.tag == 'transmission':
				pass #transmission not implemented yet
			else:
				if verbose:
					rospy.logwarn("Unknown robot element '%s'"%node.tag)
		return urdf

	@staticmethod
	def load_xml_file(filename):
		"""Parse a file to create a URDF robot structure."""
		f = open(filename, 'r')
		return URDF.parse_xml_string(f.read(), verbose)

	@staticmethod
	def load_from_parameter_server(key = 'robot_description'):
		"""
		Retrieve the robot model on the parameter server
		and parse it to create a URDF robot structure.

		Warning: this requires roscore to be running.
		"""
		import rospy
		return URDF.parse_xml_string(rospy.get_param(key), verbose)

	def add_link(self, link):
		self.elements.append(link)
		self.links[link.name] = link

	def add_joint(self, joint):
		self.elements.append(joint)
		self.joints[joint.name] = joint

		self.parent_map[ joint.child ] = (joint.name, joint.parent)
		if joint.parent in self.child_map:
			self.child_map[joint.parent].append( (joint.name, joint.child) )
		else:
			self.child_map[joint.parent] = [ (joint.name, joint.child) ]


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
		doc = Document()
		root = doc.createElement("robot")
		doc.appendChild(root)
		root.setAttribute("name", self.name)

		for element in self.elements:
			root.appendChild(element.to_xml(doc))

		return doc.toprettyxml()
