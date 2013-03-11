from urdf_parser_py.basics import *
import copy

# TODO Was a bad idea to lump elements and attributes. Split those back up.
# Clarify aggregate element types, and how to handle unconsumed attributes / elements and default values.
# Take same approach as Gazebo's SDF parser.
# Add XmlAggregateElement() to simplify design. Don't rely on `required='*'`

valueTypes = {}

def add_xml_value_type(key, value):
	assert key not in valueTypes
	valueTypes[key] = value

def get_xml_value_type(typeIn):
	""" Can wrap value types if needed """
	valueType = valueTypes.get(typeIn)
	if valueType is None:
		valueType = make_xml_value_type(typeIn)
		add_xml_value_type(typeIn, valueType)
	return valueType

def make_xml_value_type(typeIn):
	if isinstance(typeIn, XmlValueType):
		return typeIn
	elif isinstance(typeIn, str):
		if typeIn.startswith('vector'):
			extra = typeIn[6:]
			if extra:
				count = float(extra)
			else:
				count = None
			return XmlVectorType(count)
		else:
			raise Exception("Invalid value type: {}".format(typeIn))
	elif typeIn == list:
		return XmlListType()
	elif issubclass(typeIn, XmlObject):
		return XmlObjectType(typeIn)
	elif typeIn in [str, float]:
		return XmlBasicType(typeIn)
	else:
		raise Exception("Invalid type: {}".format(typeIn))

class XmlValueType(object):
	""" Primitive value type """
	def from_xml(self, node):
		return self.from_string(node.text)
	
	def to_xml(self, node, value):
		""" If type has 'to_xml', this function should expect to have it's own XML already created
		i.e., In Axis.to_sdf(self, node), 'node' would be the 'axis' element.
		TODO: Add function that makes an XML node completely independently?"""
		node.text = self.to_string(value)

class XmlBasicType(XmlValueType):
	def __init__(self, typeIn):
		self.type = typeIn
	def to_string(self, value):
		return str(value)
	def from_string(self, value):
		return self.type(value)

class XmlListType(XmlValueType):
	def to_string(self, values):
		return ' '.join(values)
	def from_string(self, text):
		return text.split()

class XmlVectorType(XmlListType):
	def __init__(self, count = None):
		self.count = count
	
	def check(self, values):
		if self.count is not None:
			assert len(values) == self.count, "Invalid vector length"
	
	def to_string(self, values):
		self.check(values)
		raw = map(str, values)
		return XmlListType.to_string(self, raw)
		
	def from_string(self, text):
		raw = XmlListType.from_string(self, text)
		self.check(raw)
		return map(float, raw)


class XmlSimpleElementType(XmlValueType):
	def __init__(self, attribute, valueType):
		self.attribute = attribute
		self.valueType = get_xml_value_type(valueType)
	def from_xml(self, node):
		text = node.get(self.attribute)
		return self.valueType.from_string(text)
	def to_xml(self, node, value):
		text = self.valueType.to_string(value)
		node.set(self.attribute, text)

class XmlObjectType(XmlValueType):
	def __init__(self, typeIn):
		self.type = typeIn
		
	def from_xml(self, node):
		obj = self.type()
		obj.load_xml(node)
		return obj
	
	def to_xml(self, node, obj):
		obj.to_xml(node)

class XmlFactoryType(XmlValueType):
	def __init__(self, name, typeMap):
		self.name = name
		self.typeMap = typeMap
		self.nameMap = {}
		for (key, value) in typeMap.iteritems():
			# Reverse lookup
			self.nameMap[value] = key
	
	def from_xml(self, node):
		typeIn = self.typeMap.get(node.tag)
		if typeIn is None:
			raise Exception("Invalid {} tag: {}".format(self.name, node.tag))
		valueType = get_xml_value_type(typeIn)
		return valueType.from_xml(node)
	
	def get_name(self, obj):
		typeIn = type(obj)
		name = self.nameMap.get(typeIn)
		if name is None:
			raise Exception("Invalid {} type: {}".format(self.name, typeIn))
		return name
	
	def to_xml(self, node, obj):
		obj.to_xml(node)


class XmlParam(object):
	""" Mirroring Gazebo's SDF api
	@param aggregate Function of the form f(obj, value) to allow it to keep track of object types when loaded from XML.
		Only called if `self.isAggregate`
	"""
	def __init__(self, name, valueType, required = True, default = None, loadAggregator = None):
		self.name = name
		self.valueType = get_xml_value_type(valueType)
		self.default = default
		self.isAggregate = False
		if required == '*':
			required = True
			self.isAggregate = True
		if loadAggregator is not None:
			assert self.isAggregate
			self.loadAggregator = loadAggregator
		if required:
			assert default is None, "Default does not make sense for a required field"
		self.required = bool(required)
	
	def set_from_xml(self, obj, node):
		raise NotImplementedError()
	
	def add_to_xml(self, obj, node):
		raise NotImplementedError()


class XmlAttribute(XmlParam):
	def __init__(self, *args, **kwargs):
		XmlParam.__init__(self, *args, **kwargs)
		assert not self.isAggregate, "Attribute cannot be list"
	
	def set_from_xml(self, obj, node, info):
		""" Node is the parent node in this case """
		if node.attrib.has_key(self.name):
			setattr(obj, self.name, self.valueType.from_string(node.attrib[self.name]))
			info.attributes.remove(self.name)
		else:
			if self.required:
				raise Exception("Required attribute not set in XML: {}".format(self.name))
			else:
				setattr(obj, self.name, self.default)
			return []
	
	def add_to_xml(self, obj, node):
		value = getattr(obj, self.name)
		# Do not set with default value if value is None
		if value is not None:
			node.set(self.name, self.valueType.to_string(value))
		elif self.required:
			raise Exception("Required attribute not set in object: {}".format(self.name))

# Simple method to preserve parsing order?
# How to encapsulate aggregate stuff in the element itself?

class XmlElement(XmlParam):
	def set_from_xml(self, obj, parent, info):
		nodes = parent.findall(self.name)
		nodeCount = len(nodes)
		# Add option if this requires a header? Like <joints> <joint/> .... </joints> ??? Not really... This would be a specific list type, not really aggregate
		if nodeCount == 1:
			node = nodes[0]
			value = self.valueType.from_xml(node)
			info.children.remove(node)
		else:
			if nodeCount > 1:
				raise Exception("Scalar element defined multiple times: {}".format(self.name))
			elif nodeCount == 0 and self.required:
				raise Exception("Required element not defined in XML: {}".format(self.name))
			value = self.default
		setattr(obj, self.name, value)
	
	def add_aggregate_from_xml(self, obj, node):
		value = self.valueType.from_xml(node)
		obj.add_aggregate(self.name, value)
				
	def add_to_xml(self, obj, parent):
		value = getattr(obj, self.name)
		assert not self.isAggregate, "This should not be called for aggregate types... Right?"
		if value is not None:
			self.add_scalar_to_xml(parent, value)
		elif self.required:
			raise Exception("Required element not defined in object: {}".format(self.name))
	
	def add_scalar_to_xml(self, parent, value):
		node = node_add(parent, self.name)
		self.valueType.to_xml(node, value)

# Add option to ensure that no extra attributes / elements are set?
# Make a 'consumption' style th

class XmlInfo(object):
	""" For keeping track of what's been parsed
	This is probably slower than just doing a for-loop on attributes and elements in order...
	But how to handle parent stuff? paramMap?
	How to handle defaults? Loop back over and see what is none?
	Would have to expect that a 'fresh' object has all fields set to None, or are empty lists
	"""
	def __init__(self, node):
		self.attributes = node.attrib.keys()
		self.children = xml_children(node)

class XmlReflection(object):
	def __init__(self, params = [], parent = None):
		self.parent = parent
		self.params = params
		self.paramMap = dict((param.name, param) for param in self.params)
		self.vars = [param.name for param in self.params]
		# Figure out which are aggregate
		self.scalars = []
		self.aggregates = []
		for param in self.params:
			if param.isAggregate:
				self.aggregates.append(param)
			else:
				self.scalars.append(param)
	
	def set_from_xml(self, obj, node, info = None):
		if info is None:
			info = XmlInfo(node)
		if self.parent:
			self.parent.set_from_xml(obj, node, info)
		for (key, value) in node.attributes:
			param = self.get_attribute(attribute)
			param.set_from_string(obj, node.attrib[attribute])
		for param in self.scalars:
			param.set_from_xml(obj, node, info)
			# Parse unconsumed child nodes
		for child in info.children:
			tag = child.tag
			param = self.paramMap.get(tag)
			if param is None:
				rospy.logwarn('Unknown tag: {}'.format(tag))
			else:
				param.add_aggregate_from_xml(obj, child)
		# TODO Complain about unconsumed stuff
	
	def add_to_xml(self, obj, node):
		if self.parent:
			self.parent.add_to_xml(obj, node)
		for param in self.scalars:
			param.add_to_xml(obj, node)
		# Now add in aggregates
		obj.add_aggregates_to_xml(node)

# Reflect basic types?
# Register variable name types, or just use some basic stuff to keep it short and simple?

class SelectiveReflection(object):
	def get_refl_vars(self):
		return vars(self).keys()

class YamlReflection(SelectiveReflection):
	def to_yaml(self):
		raw = dict((var, getattr(self, var)) for var in self.get_refl_vars())
		return to_yaml(dict(rawItems))
		
	def __str__(self):
		return yaml.dump(self.to_yaml()).rstrip() # Good idea? Will it remove other important things?

class XmlObject(YamlReflection):
	""" Raw python object for yaml / xml representation """
	XML_REFL = None
	
	def check_valid(self):
		pass
	
	def get_refl_vars(self):
		return self.XML_REFL.vars
	
	def to_xml(self, node):
		self.check_valid()
		self.XML_REFL.add_to_xml(self, node)
	
	def load_xml(self, node):
		self.XML_REFL.set_from_xml(self, node)
		self.check_valid()

	def get_aggregate_list(self, name):
		values = getattr(self, name)
		assert isinstance(values, list)
		
	def aggregate_init(self):
		""" Must be called in constructor! """
		self.aggregateOrder = []
		self.aggregateType = {} # Store this info in the loaded object??? Nah
		
	def add_aggregate(self, name, obj):
		""" NOTE: One must keep careful track of aggregate types for this system.
		Can use 'lump_aggregates()' before writing if you don't care. """
		get_aggregate_list(name).append(obj)
		self.aggregateOrder.append(obj)
		self.aggregateType[obj] = name
	
	def add_aggregates_to_xml(self, node):
		# Confusing distinction between loading code in object and reflection registry thing...
		for value in self.aggregateOrder:
			typeName = self.aggregateType[value]
			param = self.XML_REFL.paramMap[typeName]
			param.add_scalar_to_xml(node, value)
	
	def remove_aggregate(self, obj):
		self.aggregateOrder.remove(obj)
		name = self.aggregateType[obj]
		del self.aggregateType[obj]
		get_aggregate_list(name).remove(obj)
	
	def lump_aggregates(self):
		""" Put all aggregate types together, just because """
		self.aggregate_init()
		for param in self.XML_REFL.aggregates:
			for obj in self.get_aggregate_list(param.name):
				self.add_aggregate(param.name, obj)

# Really common types

add_xml_value_type('element_name', XmlSimpleElementType('name', str))
add_xml_value_type('element_value', XmlSimpleElementType('value', float))