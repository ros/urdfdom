import string
import yaml, collections
from lxml import etree
from xml.etree.ElementTree import ElementTree # Different implementations mix well it seems
import rospy

def xml_string(rootXml, addHeader = True):
    # Meh
    xmlString = etree.tostring(rootXml, pretty_print = True)
    if addHeader:
        xmlString = '<?xml version="1.0"?>\n' + xmlString
    return xmlString

def dict_sub(obj, keys):
    return dict((key, obj[key]) for key in keys)

def node_add(doc, sub):
    if sub is None:
        return None
    if type(sub) == str:
        return etree.SubElement(doc, sub)
    elif isinstance(sub, etree._Element):
        doc.append(sub) # This screws up the rest of the tree for prettyprint...
#        if verbose:
#            rospy.logwarn('Cannot add in direct xml elements...')
        return sub
    else:
        raise Exception('Invalid sub value')

def pfloat(x):
    return str(x).rstrip('.') 

def xml_children(node):
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
    elif hasattr(obj, 'to_yaml'):
        out = obj.to_yaml()
    elif isinstance(obj, etree._Element):
    	out = etree.tostring(obj, pretty_print = True)
    elif type(obj) == dict:
        out = {}
        for (var, value) in obj.iteritems():
            out[str(var)] = to_yaml(value)
    elif isinstance(obj, collections.Iterable):
        out = [to_yaml(item) for item in obj]
    else:
        out = str(obj)
    return out