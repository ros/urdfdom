from urdf_parser_py.basics import *
import copy

# TODO Make this work with decorators

# Is this reflection or serialization? I think it's serialization...
# Rename?

def reflect(cls, *args, **kwargs):
	""" Simple wrapper to add XML reflection to an xml_reflection.Object class """
	cls.XML_REFL = Reflection(*args, **kwargs)

# Rename 'to_xml' to 'dump_xml' to have paired 'load/dump', and make 'pre_dump' and 'post_load'?
# When dumping to yaml, include tag name?

# How to incorporate line number and all that jazz?
def on_error(message):
	""" What to do on an error. This can be changed to raise an exception. """
	rospy.logwarn(message);

skipDefault = True
#defaultIfMatching = True # Not implemeneted yet

# Registering Types
valueTypes = {}
valueTypePrefix = '' 

def start_namespace(namespace):
	""" Basic mechanism to prevent conflicts for string types for URDF and SDF """
	global valueTypePrefix
	valueTypePrefix = namespace + '.'

def end_namespace():
	global valueTypePrefix
	valueTypePrefix = ''

def add_type(key, value):
	if isinstance(key, str):
		key = valueTypePrefix + key
	assert key not in valueTypes
	valueTypes[key] = value

def get_type(curType):
	""" Can wrap value types if needed """
	if valueTypePrefix and isinstance(curType, str):
		# See if it exists in current 'namespace'
		curKey = valueTypePrefix + curType
		valueType = valueTypes.get(curKey)
	else:
		valueType = None
	if valueType is None:
		# Try again, in 'global' scope
		valueType = valueTypes.get(curType)
	if valueType is None:
		valueType = make_type(curType)
		add_type(curType, valueType)
	return valueType

def make_type(curType):
	if isinstance(curType, ValueType):
		return curType
	elif isinstance(curType, str):
		if curType.startswith('vector'):
			extra = curType[6:]
			if extra:
				count = float(extra)
			else:
				count = None
			return VectorType(count)
		else:
			raise Exception("Invalid value type: {}".format(curType))
	elif curType == list:
		return ListType()
	elif issubclass(curType, Object):
		return ObjectType(curType)
	elif curType in [str, float]:
		return BasicType(curType)
	else:
		raise Exception("Invalid type: {}".format(curType))

class ValueType(object):
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

class BasicType(ValueType):
	def __init__(self, curType):
		self.type = curType
	def to_string(self, value):
		return str(value)
	def from_string(self, value):
		return self.type(value)

class ListType(ValueType):
	def to_string(self, values):
		return ' '.join(values)
	def from_string(self, text):
		return text.split()
	def equals(self, aValues, bValues):
		return len(aValues) == len(bValues) and all(a == b for (a, b) in zip(aValues, bValues))

class VectorType(ListType):
	def __init__(self, count = None):
		self.count = count
	
	def check(self, values):
		if self.count is not None:
			assert len(values) == self.count, "Invalid vector length"
	
	def to_string(self, values):
		self.check(values)
		raw = map(str, values)
		return ListType.to_string(self, raw)
		
	def from_string(self, text):
		raw = ListType.from_string(self, text)
		self.check(raw)
		return map(float, raw)

class RawType(ValueType):
	""" Simple, raw XML value. Need to bugfix putting this back into a document """
	def from_xml(self, node):
		return node
	
	def to_xml(self, node, value):
		#!!! HACK Trying to insert an element at root level seems to screw up pretty printing
		children = xml_children(value)
		map(node.append, children)

class SimpleElementType(ValueType):
	def __init__(self, attribute, valueType):
		self.attribute = attribute
		self.valueType = get_type(valueType)
	def from_xml(self, node):
		text = node.get(self.attribute)
		return self.valueType.from_string(text)
	def to_xml(self, node, value):
		text = self.valueType.to_string(value)
		node.set(self.attribute, text)

class ObjectType(ValueType):
	def __init__(self, curType):
		self.type = curType
		
	def from_xml(self, node):
		obj = self.type()
		obj.load_xml(node)
		return obj
	
	def to_xml(self, node, obj):
		obj.to_xml(node)

class FactoryType(ValueType):
	def __init__(self, name, typeMap):
		self.name = name
		self.typeMap = typeMap
		self.nameMap = {}
		for (key, value) in typeMap.iteritems():
			# Reverse lookup
			self.nameMap[value] = key
	
	def from_xml(self, node):
		curType = self.typeMap.get(node.tag)
		if curType is None:
			raise Exception("Invalid {} tag: {}".format(self.name, node.tag))
		valueType = get_type(curType)
		return valueType.from_xml(node)
	
	def get_name(self, obj):
		curType = type(obj)
		name = self.nameMap.get(curType)
		if name is None:
			raise Exception("Invalid {} type: {}".format(self.name, curType))
		return name
	
	def to_xml(self, node, obj):
		obj.to_xml(node)


class Param(object):
	""" Mirroring Gazebo's SDF api
	
	@param xml_var: Xml name
		@todo If the valueType is an object with a tag defined in it's reflection, allow it to act as the default tag name?
	@param var: Python class variable name. By default it's the same as the XML name
	"""
	def __init__(self, xml_var, valueType, required = True, default = None, var = None):
		self.xml_var = xml_var
		if var is None:
			self.var = xml_var
		else:
			self.var = var
		self.type = None
		self.valueType = get_type(valueType)
		self.default = default
		if required:
			assert default is None, "Default does not make sense for a required field"
		self.required = required
		self.isAggregate = False
	
	def set_default(self):
		if self.required:
			raise Exception("Required {} not set in XML: {}".format(self.type, self.xml_var))
		elif not skipDefault:
			setattr(obj, self.var, self.default)

class Attribute(Param):
	def __init__(self, xml_var, valueType, required = True, default = None, var = None):
		Param.__init__(self, xml_var, valueType, required, default, var)
		self.type = 'attribute'
	
	def set_from_string(self, obj, value):
		""" Node is the parent node in this case """
		# Duplicate attributes cannot occur at this point
		setattr(obj, self.var, self.valueType.from_string(value))
		
	def add_to_xml(self, obj, node):
		value = getattr(obj, self.var)
		# Do not set with default value if value is None
		if value is None:
			if self.required:
				raise Exception("Required attribute not set in object: {}".format(self.var))
			elif not skipDefault:
				value = self.default
		# Allow value type to handle None?
		if value is not None:
			node.set(self.xml_var, self.valueType.to_string(value))

# Add option if this requires a header? Like <joints> <joint/> .... </joints> ??? Not really... This would be a specific list type, not really aggregate

class Element(Param):
	def __init__(self, xml_var, valueType, required = True, default = None, var = None, isRaw = False):
		Param.__init__(self, xml_var, valueType, required, default, var)
		self.type = 'element'
		self.isRaw = isRaw
		
	def set_from_xml(self, obj, node):
		value = self.valueType.from_xml(node)
		setattr(obj, self.var, value)
	
	def add_to_xml(self, obj, parent):
		value = getattr(obj, self.xml_var)
		if value is None:
			if self.required:
				raise Exception("Required element not defined in object: {}".format(self.var))
			elif not skipDefault:
				value = self.default
		if value is not None:
			self.add_scalar_to_xml(parent, value)
	
	def add_scalar_to_xml(self, parent, value):
		if self.isRaw:
			node = parent
		else:
			node = node_add(parent, self.xml_var)
		self.valueType.to_xml(node, value)


class AggregateElement(Element):
	def __init__(self, xml_var, valueType, isRaw = False):
		Element.__init__(self, xml_var, valueType, required = False, isRaw = isRaw)
		self.isAggregate = True
		
	def add_from_xml(self, obj, node):
		value = self.valueType.from_xml(node)
		obj.add_aggregate(self.xml_var, value)
	
	def set_default(self):
		pass
	

class Info:
	""" Small container for keeping track of what's been consumed """
	def __init__(self, node):
		self.attributes = node.attrib.keys()
		self.children = xml_children(node)

class Reflection(object):
	def __init__(self, params = [], parent_cls = None, tag = None):
		""" Construct a XML reflection thing
		@param parent_cls: Parent class, to use it's reflection as well.
		@param tag: Only necessary if you intend to use Object.to_xml_doc()
			This does not override the name supplied in the reflection definition thing.
		"""
		if parent_cls is not None:
			self.parent = parent_cls.XML_REFL
		else:
			self.parent = None
		self.tag = tag
		
		# Laziness for now
		attributes = []
		elements = []
		for param in params:
			if isinstance(param, Element):
				elements.append(param)
			else:
				attributes.append(param)
		
		self.vars = []
		
		self.attributes = attributes
		self.attributeMap = {}
		self.requiredAttributeNames = []
		for attribute in attributes:
			self.attributeMap[attribute.xml_var] = attribute
			self.vars.append(attribute.var)
			if attribute.required:
				self.requiredAttributeNames.append(attribute.xml_var)
		
		self.elements = []
		self.elementMap = {}
		self.requiredElementNames = []
		self.aggregates = []
		self.scalars = []
		self.scalarNames = []
		for element in elements:
			self.elementMap[element.xml_var] = element
			self.vars.append(element.var)
			if element.required:
				self.requiredElementNames.append(element.xml_var)
			if element.isAggregate:
				self.aggregates.append(element)
			else:
				self.scalars.append(element)
				self.scalarNames.append(element.xml_var)
	
	def set_from_xml(self, obj, node, info = None):
		isFinal = False
		if info is None:
			isFinal = True
			info = Info(node)
		
		if self.parent:
			self.parent.set_from_xml(obj, node, info)
		
		# Make this a map instead? Faster access? {name: isSet} ?
		unsetAttributes = self.attributeMap.keys()
		unsetScalars = copy.copy(self.scalarNames)
		
		# Better method? Queues?
		for xml_var in copy.copy(info.attributes):
			attribute = self.attributeMap.get(xml_var)
			if attribute is not None:
				value = node.attrib[xml_var]
				attribute.set_from_string(obj, value)
				unsetAttributes.remove(xml_var)
				info.attributes.remove(xml_var)
		
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
						on_error("Scalar element defined multiple times: {}".format(tag))
				info.children.remove(child)
		
		for attribute in map(self.attributeMap.get, unsetAttributes):
			attribute.set_default()
			
		for element in map(self.elementMap.get, unsetScalars):
			element.set_default()
		
		if isFinal:
			for xml_var in info.attributes:
				on_error('Unknown attribute: {}'.format(xml_var))
			for node in info.children:
				on_error('Unknown tag: {}'.format(node.tag))
	
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

class Object(YamlReflection):
	""" Raw python object for yaml / xml representation """
	XML_REFL = None
	
	def get_refl_vars(self):
		return self.XML_REFL.vars
	
	def check_valid(self):
		pass
	
	def pre_to_xml(self):
		""" If anything needs to be converted prior to dumping to xml
		i.e., getting the names of objects and such """
		pass
	
	def to_xml(self, node):
		self.check_valid()
		self.pre_to_xml()
		self.XML_REFL.add_to_xml(self, node)
	
	def to_xml_doc(self):
		tag = self.XML_REFL.tag
		assert tag is not None, "Must define 'tag' in reflection to use this function"
		doc = etree.Element(tag)
		self.to_xml(doc)
		return doc
	
	def to_xml_string(self):
		return xml_string(self.to_xml_doc())
	
	def post_load_xml(self):
		pass
	
	def load_xml(self, node):
		self.XML_REFL.set_from_xml(self, node)
		self.post_load_xml()
		self.check_valid()
		
	@classmethod
	def from_xml(cls, node):
		curType = get_type(cls)
		return curType.from_xml(node)
	
	@classmethod
	def from_xml_string(cls, xml_string):
		node = etree.fromstring(xml_string)
		return cls.from_xml(node)
	
	@classmethod
	def from_xml_file(cls, file_path):
		xml_string= open(file_path, 'r').read()
		return cls.from_xml_string(xml_string)

	# Confusing distinction between loading code in object and reflection registry thing...

	def get_aggregate_list(self, var):
		values = getattr(self, var)
		assert isinstance(values, list)
		return values
		
	def aggregate_init(self):
		""" Must be called in constructor! """
		self.aggregateOrder = []
		# Store this info in the loaded object??? Nah
		self.aggregateType = {}
		
	def add_aggregate(self, var, obj):
		""" NOTE: One must keep careful track of aggregate types for this system.
		Can use 'lump_aggregates()' before writing if you don't care. """
		self.get_aggregate_list(var).append(obj)
		self.aggregateOrder.append(obj)
		self.aggregateType[obj] = var
	
	def add_aggregates_to_xml(self, node):
		for value in self.aggregateOrder:
			typeName = self.aggregateType[value]
			element = self.XML_REFL.elementMap[typeName]
			element.add_scalar_to_xml(node, value)
	
	def remove_aggregate(self, obj):
		self.aggregateOrder.remove(obj)
		var = self.aggregateType[obj]
		del self.aggregateType[obj]
		get_aggregate_list(var).remove(obj)
	
	def lump_aggregates(self):
		""" Put all aggregate types together, just because """
		self.aggregate_init()
		for param in self.XML_REFL.aggregates:
			for obj in self.get_aggregate_list(param.var):
				self.add_aggregate(param.var, obj)

# Really common types
add_type('element_name', SimpleElementType('name', str))
add_type('element_value', SimpleElementType('value', float))

# Add in common vector types so they aren't absorbed into the namespaces
get_type('vector3')
get_type('vector4')
get_type('vector6')