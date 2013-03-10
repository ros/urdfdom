import string
import yaml, collections
from lxml import etree
from xml.etree.ElementTree import ElementTree # Different implementations mix well

def xml_string(rootXml, addHeader = True):
    # Meh
    xmlString = etree.tostring(rootXml, pretty_print = True)
    if addHeader:
        xmlString = '<?xml version="1.0"?>\n' + xmlString
    return xmlString

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
        if value and type(value[0]) in [int, float]:
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


def node_add(doc, sub):
    if sub is None:
        return None
    if type(sub) == str:
        return etree.SubElement(doc, sub)
    elif isinstance(sub, UrdfObject):
        return sub.to_xml(doc)
    elif isinstance(sub, etree._Element):
        # doc.append(sub) # This screws up the rest of the tree for prettyprint...
        if verbose:
            rospy.logwarn('Cannot add in direct xml elements...')
        return sub
    else:
        raise Exception('Invalid sub value')

def node_add_pairs(node, pairs):
    """ Multiple sub elements with text only """
    for (key, value) in pairs:
        if isinstance(value, UrdfObject):
            value.to_xml(node)
        elif value is not None:
            node_add(node, key).text = to_xml_str(value)

def node_add_dict(node, obj):
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

def children(node):
    children = node.getchildren()
    def predicate(node):
        return not isinstance(node, etree._Comment)
    return filter(predicate, children)

def to_yaml(obj):
    """ Simplify yaml representation for pretty printing """
    # Is there a better way to do this by adding a representation with yaml.Dumper?
    # Ordered dict: http://pyyaml.org/ticket/29#comment:11
    if obj is None or type(obj) in [str, unicode]:
        out = str(obj)
    elif type(obj) in [int, float]:
        return obj
    elif isinstance(obj, UrdfObject):
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