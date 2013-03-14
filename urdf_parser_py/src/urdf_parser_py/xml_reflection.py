from urdf_parser_py.basics import *
import copy

valueTypes = {}
skipDefault = True
defaultIfMatching = True # Not implemeneted yet

def xml_reflect(cls, *args, **kwargs):
	cls.XML_REFL = XmlReflection(*args, **kwargs)

# Allow this to be changed...
# How to incorporate line number and all that jazz?
def reflection_error(message):
	rospy.logwarn(message);

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
	
	def equals(self, a, b):
		return a == b

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
	def equals(self, aValues, bValues):
		return len(aValues) == len(bValues) and all(a == b for (a, b) in zip(aValues, bValues))

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
	def __init__(self, name, valueType, required = True, default = None):
		self.name = name
		self.type = None
		self.valueType = get_xml_value_type(valueType)
		self.default = default
		if required:
			assert default is None, "Default does not make sense for a required field"
		self.required = required
		self.isAggregate = False
	
	def set_default(self):
		if self.required:
			raise Exception("Required {} not set in XML: {}".format(self.type, self.name))
		elif not skipDefault:
			setattr(obj, self.name, self.default)

class XmlAttribute(XmlParam):
	def __init__(self, name, valueType, required = True, default = None):
		XmlParam.__init__(self, name, valueType, required, default)
		self.type = 'attribute'
	
	def set_from_string(self, obj, value):
		""" Node is the parent node in this case """
		# Duplicate attributes cannot occur at this point
		setattr(obj, self.name, self.valueType.from_string(value))
		
	def add_to_xml(self, obj, node):
		value = getattr(obj, self.name)
		# Do not set with default value if value is None
		if value is None:
			if self.required:
				raise Exception("Required attribute not set in object: {}".format(self.name))
			elif not skipDefault:
				value = self.default
		# Allow value type to handle None?
		if value is not None:
			node.set(self.name, self.valueType.to_string(value))

# Add option if this requires a header? Like <joints> <joint/> .... </joints> ??? Not really... This would be a specific list type, not really aggregate

class XmlElement(XmlParam):
	def __init__(self, name, valueType, required = True, default = None, isRaw = False):
		XmlParam.__init__(self, name, valueType, required, default)
		self.type = 'element'
		self.isRaw = isRaw
		
	def set_from_xml(self, obj, node):
		value = self.valueType.from_xml(node)
		setattr(obj, self.name, value)
	
	def add_to_xml(self, obj, parent):
		value = getattr(obj, self.name)
		if value is None:
			if self.required:
				raise Exception("Required element not defined in object: {}".format(self.name))
			elif not skipDefault:
				value = self.default
		if value is not None:
			self.add_scalar_to_xml(parent, value)
	
	def add_scalar_to_xml(self, parent, value):
		if self.isRaw:
			node = parent
		else:
			node = node_add(parent, self.name)
		self.valueType.to_xml(node, value)


class XmlAggregateElement(XmlElement):
	def __init__(self, name, valueType, isRaw = False):
		XmlElement.__init__(self, name, valueType, required = False, isRaw = isRaw)
		self.isAggregate = True
		
	def add_from_xml(self, obj, node):
		value = self.valueType.from_xml(node)
		obj.add_aggregate(self.name, value)
	
	def set_default(self):
		pass
	

class XmlInfo:
	def __init__(self, node):
		self.attributes = node.attrib.keys()
		self.children = xml_children(node)

class XmlReflection(object):
	def __init__(self, params = [], parent = None):
		self.parent = parent
		
		# Laziness for now
		attributes = []
		elements = []
		for param in params:
			if isinstance(param, XmlElement):
				elements.append(param)
			else:
				attributes.append(param)
		
		self.vars = []
		
		self.attributes = attributes
		self.attributeMap = {}
		self.requiredAttributeNames = []
		for attribute in attributes:
			self.attributeMap[attribute.name] = attribute
			self.vars.append(attribute.name)
			if attribute.required:
				self.requiredAttributeNames.append(attribute.name)
		
		self.elements = []
		self.elementMap = {}
		self.requiredElementNames = []
		self.aggregates = []
		self.scalars = []
		self.scalarNames = []
		for element in elements:
			self.elementMap[element.name] = element
			self.vars.append(element.name)
			if element.required:
				self.requiredElementNames.append(element.name)
			if element.isAggregate:
				self.aggregates.append(element)
			else:
				self.scalars.append(element)
				self.scalarNames.append(element.name)
	
	def set_from_xml(self, obj, node, info = None):
		isFinal = False
		if info is None:
			isFinal = True
			info = XmlInfo(node)
		
		if self.parent:
			self.parent.set_from_xml(obj, node, info)
		
		# Make this a map instead? Faster access? {name: isSet} ?
		unsetAttributes = self.attributeMap.keys()
		unsetScalars = copy.copy(self.scalarNames)
		
		# Better method? Queues?
		for name in copy.copy(info.attributes):
			attribute = self.attributeMap.get(name)
			if attribute is not None:
				value = node.attrib[name]
				attribute.set_from_string(obj, value)
				unsetAttributes.remove(name)
				info.attributes.remove(name)
		
		# Parse unconsumed nodes
		for child in copy.copy(info.children):
			tag = child.tag
			element = self.elementMap.get(tag)
			if element is not None:
				if element.isAggregate:
					element.add_from_xml(obj, child)
				else:
					if tag in unsetScalars:
						element.set_from_xml(obj, child)
						unsetScalars.remove(tag)
					else:
						reflection_error("Scalar element defined multiple times: {}".format(tag))
				info.children.remove(child)
		
		for attribute in map(self.attributeMap.get, unsetAttributes):
			attribute.set_default()
			
		for element in map(self.elementMap.get, unsetScalars):
			element.set_default()
		
		if isFinal:
			for name in info.attributes:
				reflection_error('Unknown attribute: {}'.format(name))
			for node in info.children:
				reflection_error('Unknown tag: {}'.format(node.tag))
	
	def add_to_xml(self, obj, node):
		if self.parent:
			self.parent.add_to_xml(obj, node)
		for attribute in self.attributes:
			attribute.add_to_xml(obj, node)
		for element in self.scalars:
			element.add_to_xml(obj, node)
		# Now add in aggregates
		if self.aggregates:
			obj.add_aggregates_to_xml(node)

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
		return values
		
	def aggregate_init(self):
		""" Must be called in constructor! """
		self.aggregateOrder = []
		self.aggregateType = {} # Store this info in the loaded object??? Nah
		
	def add_aggregate(self, name, obj):
		""" NOTE: One must keep careful track of aggregate types for this system.
		Can use 'lump_aggregates()' before writing if you don't care. """
		self.get_aggregate_list(name).append(obj)
		self.aggregateOrder.append(obj)
		self.aggregateType[obj] = name
	
	def add_aggregates_to_xml(self, node):
		# Confusing distinction between loading code in object and reflection registry thing...
		for value in self.aggregateOrder:
			typeName = self.aggregateType[value]
			element = self.XML_REFL.elementMap[typeName]
			element.add_scalar_to_xml(node, value)
	
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