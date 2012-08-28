import string
import rospy
import xml.dom.minidom
from xml.dom.minidom import Document

def reindent(s, numSpaces):
    """Reindent a string for tree structure pretty printing."""
    s = string.split(s, '\n')
    s = [(numSpaces * ' ') + line for line in s]
    s = string.join(s, '\n')
    return s

def add(doc, base, element):
    if element is not None:
        base.appendChild( element.to_xml(doc) )

def pfloat(x):
    return "{0}".format(x).rstrip('.')

def set_attribute(node, name, value):
    if value is None:
        return
    if type([]) == type(value) or type(()) == type(value):
        value = " ".join( [pfloat(a) for a in value] )
    elif type(value) == type(0.0):
        value = pfloat(value)
    elif type(value) != type(''):
        value = str(value)
    node.setAttribute(name, value)

def short(doc, name, key, value):
    element = doc.createElement(name)
    set_attribute(element, key, value)
    return element

def children(node):
    children = []
    for child in node.childNodes:
        if child.nodeType is node.TEXT_NODE \
                or child.nodeType is node.COMMENT_NODE:
            continue
        else:
            children.append(child)
    return children

class Collision(object):
    def __init__(self, geometry=None, origin=None):
        self.geometry = geometry
        self.origin = origin

    @staticmethod
    def parse(node, verbose=True):
        c = Collision()
        for child in children(node):
            if child.localName == 'geometry':
                c.geometry = Geometry.parse(child, verbose)
            elif child.localName == 'origin':
                c.origin = Pose.parse(child)
            else:
                if verbose:
                    rospy.logwarn("Unknown collision element '%s'"%child.localName)
        return c

    def to_xml(self, doc):
        xml = doc.createElement("collision")
        add(doc, xml, self.geometry)
        add(doc, xml, self.origin)
        return xml

    def __str__(self):
        s =  "Origin:\n{0}\n".format(reindent(str(self.origin), 1))
        s += "Geometry:\n{0}\n".format(reindent(str(self.geometry), 1))
        return s

class Color(object):
    def __init__(self, r=0.0, g=0.0, b=0.0, a=0.0):
        self.rgba=(r,g,b,a)

    @staticmethod
    def parse(node):
        rgba = node.getAttribute("rgba").split()
        (r,g,b,a) = [ float(x) for x in rgba ]
        return Color(r,g,b,a)

    def to_xml(self, doc):
        xml = doc.createElement("color")
        set_attribute(xml, "rgba", self.rgba)
        return xml

    def __str__(self):
        return "r: {0}, g: {1}, b: {2}, a: {3},".format(
            self.r, self.g, self.b, self.a)

class Dynamics(object):
    def __init__(self, damping=None, friction=None):
        self.damping = damping
        self.friction = friction

    @staticmethod
    def parse(node):
        d = Dynamics()
        if node.hasAttribute('damping'):
            d.damping = node.getAttribute('damping')
        if node.hasAttribute('friction'):
            d.friction = node.getAttribute('friction')
        return d

    def to_xml(self, doc):
        xml = doc.createElement('dynamics')
        set_attribute(xml, 'damping', self.damping)
        set_attribute(xml, 'friction', self.friction)
        return xml


    def __str__(self):
        return "Damping: {0}\nFriction: {1}\n".format(self.damping,
                                                      self.friction)

class Geometry(object):
    def __init__(self):
        None

    @staticmethod
    def parse(node, verbose=True):
        shape = children(node)[0]
        if shape.localName=='box':
            return Box.parse(shape)
        elif shape.localName=='cylinder':
            return Cylinder.parse(shape)
        elif shape.localName=='sphere':
            return Sphere.parse(shape)
        elif shape.localName=='mesh':
            return Mesh.parse(shape)
        else:
            if verbose:
                rospy.logwarn("Unknown shape %s"%child.localName)

    def __str__(self):
        return "Geometry abstract class"

class Box(Geometry):
    def __init__(self, dims=None):
        if dims is None:
            self.dims = None
        else:
            self.dims = (dims[0], dims[1], dims[2])

    @staticmethod
    def parse(node):
        dims = node.getAttribute('size').split()
        return Box([float(a) for a in dims])

    def to_xml(self, doc):
        xml = doc.createElement("box")
        set_attribute(xml, "size", self.dims)
        geom = doc.createElement('geometry')
        geom.appendChild(xml)
        return geom

    def __str__(self):
        return "Dimension: {0}".format(self.dims)


class Cylinder(Geometry):
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length

    @staticmethod
    def parse(node):
        r = node.getAttribute('radius')
        l = node.getAttribute('length')
        return Cylinder(float(r), float(l))

    def to_xml(self, doc):
        xml = doc.createElement("cylinder")
        set_attribute(xml, "radius", self.radius)
        set_attribute(xml, "length", self.length)
        geom = doc.createElement('geometry')
        geom.appendChild(xml)
        return geom

    def __str__(self):
        return "Radius: {0}\nLength: {1}".format(self.radius,
                                                 self.length)

class Sphere(Geometry):
    def __init__(self, radius=0.0):
        self.radius = radius

    @staticmethod
    def parse(node):
        r = node.getAttribute('radius')
        return Sphere(float(r))

    def to_xml(self, doc):
        xml = doc.createElement("sphere")
        set_attribute(xml, "radius", self.radius)
        geom = doc.createElement('geometry')
        geom.appendChild(xml)
        return geom

    def __str__(self):
        return "Radius: {0}".format(self.radius)


class Mesh(Geometry):
    def __init__(self, filename=None, scale=None):
        self.filename = filename
        self.scale = scale

    @staticmethod
    def parse(node):
        fn = node.getAttribute('filename')
        s = node.getAttribute('scale')
        if s == "":
            s = None
        else:
            xyz = node.getAttribute('scale').split()
            scale = map(float, xyz)
        return Mesh(fn, s)

    def to_xml(self, doc):
        xml = doc.createElement("mesh")
        set_attribute(xml, "filename", self.filename)
        set_attribute(xml, "scale", self.scale)
        geom = doc.createElement('geometry')
        geom.appendChild(xml)
        return geom

    def __str__(self):
        return "Filename: {0}\nScale: {1}".format(self.filename, self.scale)


class Inertial(object):
    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0,
                 mass=0.0, origin=None):
        self.matrix = {}
        self.matrix['ixx'] = ixx
        self.matrix['ixy'] = iyy
        self.matrix['ixz'] = ixz
        self.matrix['iyy'] = iyy
        self.matrix['iyz'] = iyz
        self.matrix['izz'] = izz
        self.mass = mass
        self.origin = origin

    @staticmethod
    def parse(node):
        inert = Inertial()
        for child in children(node):
            if child.localName=='inertia':
                for v in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
                    inert.matrix[v] = float(child.getAttribute(v))
            elif child.localName=='mass':
                inert.mass = float(child.getAttribute('value'))
            elif child.localName == 'origin':
                inert.origin = Pose.parse(child)
        return inert

    def to_xml(self, doc):
        xml = doc.createElement("inertial")

        xml.appendChild(short(doc, "mass", "value", self.mass))

        inertia = doc.createElement("inertia")
        for (n,v) in self.matrix.items():
            set_attribute(inertia, n, v)
        xml.appendChild(inertia)

        add(doc, xml, self.origin)
        return xml

    def __str__(self):
        s =  "Origin:\n{0}\n".format(reindent(str(self.origin), 1))
        s += "Mass: {0}\n".format(self.mass)
        s += "ixx: {0}\n".format(self.matrix['ixx'])
        s += "ixy: {0}\n".format(self.matrix['ixy'])
        s += "ixz: {0}\n".format(self.matrix['ixz'])
        s += "iyy: {0}\n".format(self.matrix['iyy'])
        s += "iyz: {0}\n".format(self.matrix['iyz'])
        s += "izz: {0}\n".format(self.matrix['izz'])
        return s


class Joint(object):
    UNKNOWN = 'unknown'
    REVOLUTE = 'revolute'
    CONTINUOUS = 'continuous'
    PRISMATIC = 'prismatic'
    FLOATING = 'floating'
    PLANAR = 'planar'
    FIXED = 'fixed'


    def __init__(self, name, parent, child, joint_type, axis=None, origin=None,
                 limits=None, dynamics=None, safety=None, calibration=None,
                 mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.joint_type = joint_type
        self.axis = axis
        self.origin = origin
        self.limits = limits
        self.dynamics = dynamics
        self.safety = safety
        self.calibration = calibration
        self.mimic = mimic

    @staticmethod
    def parse(node, verbose=True):
        joint = Joint(node.getAttribute('name'), None, None,
                      node.getAttribute('type'))
        for child in children(node):
            if child.localName == 'parent':
                joint.parent = child.getAttribute('link')
            elif child.localName == 'child':
                joint.child = child.getAttribute('link')
            elif child.localName == 'axis':
                joint.axis = child.getAttribute('xyz')
            elif child.localName == 'origin':
                joint.origin = Pose.parse(child)
            elif child.localName == 'limit':
                joint.limits = JointLimit.parse(child)
            elif child.localName == 'dynamics':
                joint.dynamics = Dynamics.parse(child)
            elif child.localName == 'safety_controller':
                joint.safety = SafetyController.parse(child)
            elif child.localName == 'calibration':
                joint.calibration = JointCalibration.parse(child)
            elif child.localName == 'mimic':
                joint.mimic = JointMimic.parse(child)
            else:
                if verbose:
                    rospy.logwarn("Unknown joint element '%s'"%child.localName)
        return joint

    def to_xml(self, doc):
        xml = doc.createElement("joint")
        set_attribute(xml, "name", self.name)
        set_attribute(xml, "type", self.joint_type)
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


    def __str__(self):
        s = "Name: {0}\n".format(self.name)

        s += "Child link name: {0}\n".format(self.child)
        s += "Parent link name: {0}\n".format(self.parent)

        if self.joint_type == Joint.UNKNOWN:
            s += "Type: unknown\n"
        elif self.joint_type == Joint.REVOLUTE:
            s += "Type: revolute\n"
        elif self.joint_type == Joint.CONTINUOUS:
            s += "Type: continuous\n"
        elif self.joint_type == Joint.PRISMATIC:
            s += "Type: prismatic\n"
        elif self.joint_type == Joint.FLOATING:
            s += "Type: floating\n"
        elif self.joint_type == Joint.PLANAR:
            s += "Type: planar\n"
        elif self.joint_type == Joint.FIXED:
            s += "Type: fixed\n"
        else:
            rospy.logwarn("unknown joint type")

        s +=  "Axis: {0}\n".format(self.axis)
        s +=  "Origin:\n{0}\n".format(reindent(str(self.origin), 1))
        s += "Limits:\n"
        s += reindent(str(self.limits), 1) + "\n"
        s += "Dynamics:\n"
        s += reindent(str(self.dynamics), 1) + "\n"
        s += "Safety:\n"
        s += reindent(str(self.safety), 1) + "\n"
        s += "Calibration:\n"
        s += reindent(str(self.calibration), 1) + "\n"
        s += "Mimic:\n"
        s += reindent(str(self.mimic), 1) + "\n"
        return s


#FIXME: we are missing the reference position here.
class JointCalibration(object):
    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling

    @staticmethod
    def parse(node):
        jc = JointCalibration()
        if node.hasAttribute('rising'):
            jc.rising = float( node.getAttribute('rising') )
        if node.hasAttribute('falling'):
            jc.falling = float( node.getAttribute('falling') )
        return jc

    def to_xml(self, doc):
        xml = doc.createElement('calibration')
        set_attribute(xml, 'rising', self.rising)
        set_attribute(xml, 'falling', self.falling)
        return xml

    def __str__(self):
        s = "Raising: {0}\n".format(self.rising)
        s += "Falling: {0}\n".format(self.falling)
        return s

class JointLimit(object):
    def __init__(self, effort, velocity, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper

    @staticmethod
    def parse(node):
        jl = JointLimit( float( node.getAttribute('effort') ) ,
                         float( node.getAttribute('velocity')))
        if node.hasAttribute('lower'):
            jl.lower = float( node.getAttribute('lower') )
        if node.hasAttribute('upper'):
            jl.upper = float( node.getAttribute('upper') )
        return jl

    def to_xml(self, doc):
        xml = doc.createElement('limit')
        set_attribute(xml, 'effort', self.effort)
        set_attribute(xml, 'velocity', self.velocity)
        set_attribute(xml, 'lower', self.lower)
        set_attribute(xml, 'upper', self.upper)
        return xml

    def __str__(self):
        s = "Effort: {0}\n".format(self.effort)
        s += "Lower: {0}\n".format(self.lower)
        s += "Upper: {0}\n".format(self.upper)
        s += "Velocity: {0}\n".format(self.velocity)
        return s

#FIXME: we are missing __str__ here.
class JointMimic(object):
    def __init__(self, joint_name, multiplier=None, offset=None):
        self.joint_name = joint_name
        self.multiplier = multiplier
        self.offset = offset

    @staticmethod
    def parse(node):
        mimic = JointMimic( node.getAttribute('joint') )
        if node.hasAttribute('multiplier'):
            mimic.multiplier = float( node.getAttribute('multiplier') )
        if node.hasAttribute('offset'):
            mimic.offset = float( node.getAttribute('offset') )
        return mimic

    def to_xml(self, doc):
        xml = doc.createElement('mimic')
        set_attribute(xml, 'joint', self.joint_name)
        set_attribute(xml, 'multiplier', self.multiplier)
        set_attribute(xml, 'offset', self.offset)
        return xml

class Link(object):
    def __init__(self, name, visual=None, inertial=None, collision=None):
        self.name = name
        self.visual = visual
        self.inertial=inertial
        self.collision=collision

    @staticmethod
    def parse(node, verbose=True):
        link = Link(node.getAttribute('name'))
        for child in children(node):
            if child.localName == 'visual':
                link.visual = Visual.parse(child, verbose)
            elif child.localName == 'collision':
                link.collision = Collision.parse(child, verbose)
            elif child.localName == 'inertial':
                link.inertial = Inertial.parse(child)
            else:
                if verbose:
                    rospy.logwarn("Unknown link element '%s'"%child.localName)
        return link

    def to_xml(self, doc):
        xml = doc.createElement("link")
        xml.setAttribute("name", self.name)
        add( doc, xml, self.visual)
        add( doc, xml, self.collision)
        add( doc, xml, self.inertial)
        return xml

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "Inertial:\n"
        s += reindent(str(self.inertial), 1) + "\n"
        s += "Collision:\n"
        s += reindent(str(self.collision), 1) + "\n"
        return s

class Material(object):
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    @staticmethod
    def parse(node, verbose=True):
        material = Material()
        if node.hasAttribute('name'):
            material.name = node.getAttribute('name')
        for child in children(node):
            if child.localName == 'color':
                material.color = Color.parse(child)
            elif child.localName == 'texture':
                material.texture = child.getAttribute('filename')
            else:
                if verbose:
                    rospy.logwarn("Unknown material element '%s'"%child.localName)

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

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "Color: {0}\n".format(self.color)
        s += "Texture:\n"
        s += reindent(str(self.texture), 1)
        return s


class Pose(object):
    def __init__(self, position=None, rotation=None):
        self.position = position
        self.rotation = rotation

    @staticmethod
    def parse(node):
        pose = Pose()
        if node.hasAttribute("xyz"):
            xyz = node.getAttribute('xyz').split()
            pose.position = map(float, xyz)
        if node.hasAttribute("rpy"):
            rpy = node.getAttribute('rpy').split()
            pose.rotation = map(float, rpy)
        return pose

    def to_xml(self, doc):
        xml = doc.createElement("origin")
        set_attribute(xml, 'xyz', self.position)
        set_attribute(xml, 'rpy', self.rotation)
        return xml


    def __str__(self):
        return "Position: {0}\nRotation: {1}".format(self.position,
                                                     self.rotation)


class SafetyController(object):
    def __init__(self, velocity, position=None, lower=None, upper=None):
        self.velocity = velocity
        self.position = position
        self.lower = lower
        self.upper = upper

    @staticmethod
    def parse(node):
        sc = SafetyController( float(node.getAttribute('k_velocity')) )
        if node.hasAttribute('soft_lower_limit'):
            sc.lower = float( node.getAttribute('soft_lower_limit') )
        if node.hasAttribute('soft_upper_limit'):
            sc.upper = float( node.getAttribute('soft_upper_limit') )
        if node.hasAttribute('k_position'):
            sc.position = float( node.getAttribute('k_position') )
        return sc

    def to_xml(self, doc):
        xml = doc.createElement('safety_controller')
        set_attribute(xml, 'k_velocity', self.velocity)
        set_attribute(xml, 'k_position', self.position)
        set_attribute(xml, 'soft_upper_limit', self.upper)
        set_attribute(xml, 'soft_lower_limit', self.lower)
        return xml

    def __str__(self):
        s = "Safe lower limit: {0}\n".format(self.lower)
        s += "Safe upper limit: {0}\n".format(self.upper)
        s += "K position: {0}\n".format(self.position)
        s += "K velocity: {0}\n".format(self.velocity)
        return s


class Visual(object):
    def __init__(self, geometry=None, material=None, origin=None):
        self.geometry = geometry
        self.material = material
        self.origin = origin

    @staticmethod
    def parse(node, verbose=True):
        v = Visual()
        for child in children(node):
            if child.localName == 'geometry':
                v.geometry = Geometry.parse(child, verbose)
            elif child.localName == 'origin':
                v.origin = Pose.parse(child)
            elif child.localName == 'material':
                v.material = Material.parse(child, verbose)
            else:
                if verbose:
                    rospy.logwarn("Unknown visual element '%s'"%child.localName)
        return v

    def to_xml(self, doc):
        xml = doc.createElement("visual")
        add( doc, xml, self.geometry )
        add( doc, xml, self.origin )
        add( doc, xml, self.material )
        return xml

    def __str__(self):
        s =  "Origin:\n{0}\n".format(reindent(str(self.origin), 1))
        s += "Geometry:\n"
        s += reindent(str(self.geometry), 1) + "\n"
        s += "Material:\n"
        s += reindent(str(self.material), 1) + "\n"
        return s

class URDF(object):
    def __init__(self, name=""):
        self.name = name
        self.elements = []
        self.links = {}
        self.joints = {}
        self.materials = {}

        self.parent_map = {}
        self.child_map = {}

    @staticmethod
    def parse_xml_string(xml_string, verbose=True):
        """Parse a string to create a URDF robot structure."""
        urdf = URDF()
        base = xml.dom.minidom.parseString(xml_string)
        robot = children(base)[0]
        urdf.name = robot.getAttribute('name')

        for node in children(robot):
            if node.nodeType is node.TEXT_NODE:
                continue
            if node.localName == 'joint':
                urdf.add_joint( Joint.parse(node, verbose) )
            elif node.localName == 'link':
                urdf.add_link( Link.parse(node, verbose) )
            elif node.localName == 'material':
                urdf.elements.append( Material.parse(node, verbose) )
            elif node.localName == 'gazebo':
                None #Gazebo not implemented yet
            elif node.localName == 'transmission':
                None #transmission not implemented yet
            else:
                if verbose:
                    rospy.logwarn("Unknown robot element '%s'"%node.localName)
        return urdf

    @staticmethod
    def load_xml_file(filename, verbose=True):
        """Parse a file to create a URDF robot structure."""
        f = open(filename, 'r')
        return URDF.parse_xml_string(f.read(), verbose)

    @staticmethod
    def load_from_parameter_server(key = 'robot_description', verbose=True):
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


    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "\n"
        s += "Links:\n"
        if len(self.links) == 0:
            s += "None\n"
        else:
            for k, v in self.links.iteritems():
                s += "- Link '{0}':\n{1}\n".format(k, reindent(str(v), 1))
        s += "\n"
        s += "Joints:\n"
        if len(self.joints) == 0:
            s += "None\n"
        else:
            for k, v in self.joints.iteritems():
                s += "- Joint '{0}':\n{1}\n".format(k, reindent(str(v), 1))
        s += "\n"
        s += "Materials:\n"
        if len(self.materials) == 0:
            s += "None\n"
        else:
            for k, v in self.materials.iteritems():
                s += "- Material '{0}':\n{1}\n".format(k, reindent(str(v), 1))

        return s

        self.name = name
        self.elements = []
        self.links = {}
        self.joints = {}
        self.materials = {}

        self.parent_map = {}
        self.child_map = {}
