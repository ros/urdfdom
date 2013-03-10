from urdf_parser_py.basics import *

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

class YamlObject(SelectiveReflection):
	def to_yaml(self):
		return to_yaml(dict(self.get_refl_vars()))
		
	def __str__(self):
		return yaml.dump(self.to_yaml()).rstrip() # Good idea? Will it remove other important things?


def from_xml_string(typeIn, text):
	if isinstance(typeIn, str):
		if typeIn.startswith('vector'):
			value = from_xml_vector(text)
			vector_assert(typeIn, value)
		else:
			raise Exception("Invalid XML string type")
	else:
		if typeIn == list:
			return from_xml_list(value)
		else:
			return typeIn(value)

def to_xml_string(typeIn, value):
	if isinstance(typeIn, str):
		if typeIn.startswith('vector'):
			vector_assert(typeIn, value)
			value = to_xml_vector(value)
		else:
			raise Exception("Invalid XML string type")
	else:
		if typeIn == list:
			return from_xml_list(value)
		else:
			return str(value)

def vector_assert(typeIn, value):
	assert typeIn.startswith('vector')
	extra = typeIn[6:]
	if extra:
		count = float(extra)
		assert len(value) == count

def XmlValue(object):
	def __init__(self, typeIn):
		self.type = type
	
	def from_xml_node(self, node):
		if hasattr(typeIn, 'from_xml'):
			return typeIn.from_xml(node)
		elif hasattr(typeIn, 'load_xml'):
			obj = typeIn()
			obj.load_xml(node)
			return obj

def from_xml_node(typeIn, node):
	if isinstance(typeIn, type):
	# This is not robust... Oh well
	return from_xml_string(typeIn, node.text)

def to_xml_node(parent, typeIn, value):
	""" If type has 'to_xml', this function should expect to have it's own XML already created
	i.e., In Axis.to_sdf(self, node), 'node' would be the 'axis' element.
	TODO: Add function that makes an XML node completely independently?"""
	if isinstance(typeIn, type) and hasattr(typeIn, 'to_xml'):
		typeIn.to_xml(value, parent)
	else:
		parent.text = to_xml_string(typeIn, value)

class XmlParam(object):
	""" Mirroring Gazebo's SDF api """
	def __init__(self, name, valueType, required = True, default = None):
		self.name = name
		if not isinstance(valueType, XmlValueType):
			valueType = XmlSimpleValueType(valueType)
		self.valueType = valueType
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
			node.set(self.name, self.valueType.to_string(self.type, node, value))
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
				value = self.valueType.from_node(node)
				values.append(value)
		else:
			if nodeCount == 1:
				value = self.valueType.from_node(nodes[0])
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
				self.valueType.to_node(node, value)
		else:
			if value is not None:
				node = node_add(parent, self.name)
				to_xml_node(node, self.type, value)
			elif self.required:
				raise Exception("Required element not defined in object: {}".format(self.name))				
				
class XmlReflection(object):
	def __init__(self, parent = None, params = []):
		self.parent = parent
		self.params = params
		self.vars = []
		for param in self.params:
			self.vars.append(param.name)
	
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

class XmlObject(YamlObject):
	""" Raw python object for yaml / xml representation """
	XML_REFL = None
	
	def to_xml(self, node):
		self.__class__.XML_REFL.add_to_xml(self, node)
	
	def load_xml(self, node):
		self.__class__.XML_REFL.set_from_xml(self, node)