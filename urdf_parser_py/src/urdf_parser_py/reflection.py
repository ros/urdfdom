from urdf_parser_py.basics import *

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
	""" Mirroring Gazebo's SDF api """
	def __init__(self, name, valueType, required = True, default = None):
		self.name = name
		self.valueType = get_xml_value_type(valueType)
		self.default = default
		self.isList = False
		if required == '*':
			required = True
			self.isList = True
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
		assert not self.isList, "Attribute cannot be list"
	
	def set_from_xml(self, obj, node):
		""" Node is the parent node in this case """
		if node.attrib.has_key(self.name):
			setattr(obj, self.name, self.valueType.from_string(node.attrib[self.name]))
		else:
			if self.required:
				raise Exception("Required attribute not set in XML: {}".format(self.name))
			else:
				setattr(obj, self.name, self.default)
	
	def add_to_xml(self, obj, node):
		value = getattr(obj, self.name)
		# Do not set with default value if value is None
		if value is not None:
			node.set(self.name, self.valueType.to_string(value))
		elif self.required:
			raise Exception("Required attribute not set in object: {}".format(self.name))

# Simple method to preserve parsing order?

class XmlElement(XmlParam):
	def set_from_xml(self, obj, parent):
		nodes = parent.findall(self.name)
		nodeCount = len(nodes)
		# Add option if this requires a header?
		if self.isList:
			values = getattr(obj, self.name)
			assert isinstance(values, list)
			for node in nodes:
				value = self.valueType.from_xml(node)
				values.append(value)
		else:
			if nodeCount == 1:
				value = self.valueType.from_xml(nodes[0])
			else:
				if nodeCount > 1:
					raise Exception("Element that should be single defined multiple times: {}".format(self.name))
				elif nodeCount == 0 and self.required:
					raise Exception("Required element not defined in XML: {}".format(self.name))
				value = self.default
			setattr(obj, self.name, value)
				
	def add_to_xml(self, obj, parent):
		value = getattr(obj, self.name)
		if self.isList:
			values = value
			assert isinstance(values, list)
			for value in values:
				node = node_add(parent, self.name)
				self.valueType.to_xml(node, value)
		else:
			if value is not None:
				node = node_add(parent, self.name)
				self.valueType.to_xml(node, value)
			elif self.required:
				raise Exception("Required element not defined in object: {}".format(self.name))				

# Add option to ensure that no extra attributes / elements are set?
# Make a 'consumption' style thing? Or just a for-loop?

class XmlReflection(object):
	def __init__(self, params = [], parent = None):
		self.parent = parent
		self.params = params
		self.vars = [param.name for param in self.params]
	
	def set_from_xml(self, obj, node):
		if self.parent:
			self.parent.set_from_xml(obj, node)
		for param in self.params:
			param.set_from_xml(obj, node)
	
	def add_to_xml(self, obj, node):
		if self.parent:
			self.parent.add_to_xml(obj, node)
		for param in self.params:
			param.add_to_xml(obj, node)

# Reflect basic types?
# Register variable name types, or just use some basic stuff to keep it short and simple?

class SelectiveReflection(object):
	def get_refl_vars(self, var_list_var = 'var_list'):
		all = vars(self)
		var_list = getattr(self, var_list_var, None)
		if var_list is not None:
			out = [[var, all[var]] for var in var_list]
		else:
			out = all.items() 
		return out

class YamlReflection(SelectiveReflection):
	def to_yaml(self):
		return to_yaml(dict(self.get_refl_vars()))
		
	def __str__(self):
		return yaml.dump(self.to_yaml()).rstrip() # Good idea? Will it remove other important things?

class XmlObject(object):
	""" Raw python object for yaml / xml representation """
	XML_REFL = None
	
	def get_refl_vars(self):
		return self.XML_REFL.vars
	
	def to_xml(self, node):
		self.XML_REFL.add_to_xml(self, node)
	
	def load_xml(self, node):
		self.XML_REFL.set_from_xml(self, node)

# Really common types

add_xml_value_type('element_name', XmlSimpleElementType('name', str))
add_xml_value_type('element_value', XmlSimpleElementType('value', float))