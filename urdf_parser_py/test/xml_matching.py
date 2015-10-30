import xml.dom
import re
import sys

# regex to match whitespace
whitespace = re.compile(r'\s+')

def all_attributes_match(a, b):
    if len(a.attributes) != len(b.attributes):
        print("Different number of attributes")
        return False
    a_atts = [(a.attributes.item(i).name, a.attributes.item(i).value) for i in range(len(a.attributes))]
    b_atts = [(b.attributes.item(i).name, b.attributes.item(i).value) for i in range(len(b.attributes))]
    a_atts.sort()
    b_atts.sort()

    for i in range(len(a_atts)):
        if a_atts[i][0] != b_atts[i][0]:
            print("Different attribute names: %s and %s" % (a_atts[i][0], b_atts[i][0]))
            return False
        try:
            if abs(float(a_atts[i][1]) - float(b_atts[i][1])) > 1.0e-9:
                print("Different attribute values: %s and %s" % (a_atts[i][1], b_atts[i][1]))
                return False
        except ValueError:  # Attribute values aren't numeric
            if a_atts[i][1] != b_atts[i][1]:
                print("Different attribute values: %s and %s" % (a_atts[i][1], b_atts[i][1]))
                return False

    return True


def text_matches(a, b):
    a_norm = whitespace.sub(' ', a)
    b_norm = whitespace.sub(' ', b)
    if a_norm.strip() == b_norm.strip(): return True
    print("Different text values: '%s' and '%s'" % (a, b))
    return False


def nodes_match(a, b, ignore_nodes):
    if not a and not b:
        return True
    if not a or not b:
        return False

    if a.nodeType != b.nodeType:
        print("Different node types: %s and %s" % (a, b))
        return False

    # compare text-valued nodes
    if a.nodeType in [xml.dom.Node.TEXT_NODE,
                      xml.dom.Node.CDATA_SECTION_NODE,
                      xml.dom.Node.COMMENT_NODE]:
        return text_matches(a.data, b.data)

    # ignore all other nodes except ELEMENTs
    if a.nodeType != xml.dom.Node.ELEMENT_NODE:
        return True

    # compare ELEMENT nodes
    if a.nodeName != b.nodeName:
        print("Different element names: %s and %s" % (a.nodeName, b.nodeName))
        return False

    if not all_attributes_match(a, b):
        return False

    a = a.firstChild
    b = b.firstChild
    while a or b:
        # ignore whitespace-only text nodes
        # we could have several text nodes in a row, due to replacements
        while (a and
               ((a.nodeType in ignore_nodes) or
                (a.nodeType == xml.dom.Node.TEXT_NODE and whitespace.sub('', a.data) == ""))):
            a = a.nextSibling
        while (b and
               ((b.nodeType in ignore_nodes) or
                (b.nodeType == xml.dom.Node.TEXT_NODE and whitespace.sub('', b.data) == ""))):
            b = b.nextSibling

        if not nodes_match(a, b, ignore_nodes):
            return False

        if a: a = a.nextSibling
        if b: b = b.nextSibling

    return True


def xml_matches(a, b, ignore_nodes=[]):
    if isinstance(a, str):
        return xml_matches(xml.dom.minidom.parseString(a).documentElement, b, ignore_nodes)
    if isinstance(b, str):
        return xml_matches(a, xml.dom.minidom.parseString(b).documentElement, ignore_nodes)
    if a.nodeType == xml.dom.Node.DOCUMENT_NODE:
        return xml_matches(a.documentElement, b, ignore_nodes)
    if b.nodeType == xml.dom.Node.DOCUMENT_NODE:
        return xml_matches(a, b.documentElement, ignore_nodes)

    if not nodes_match(a, b, ignore_nodes):
        print("Match failed:")
        a.writexml(sys.stdout)
        print()
        print('=' * 78)
        b.writexml(sys.stdout)
        print()
        return False
    return True
