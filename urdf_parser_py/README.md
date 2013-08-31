# urdf_parser_py

## Development Setup

You must manually run `setup.py`. For catkin development, you can install to $ws/../build/lib/pythonX.Y/dist-packages via

	devel_prefix=$(cd $(catkin_find --first-only)/.. && pwd)
	cd ../urdf_parser_py
	python setup.py install --install-layout deb --prefix $devel_prefix

Not yet sure how to get it to generate catkin-like development installs, which uses `__init__.py` to point to the development source directory.

## Authors

*	Thomas Moulard - `urdfpy` implementation, integration
*	David Lu - `urdf_python` implementation, integration
*	Kelsey Hawkins - `urdf_parser_python` implementation, integration
*	Antonio El Khoury - bugfixes
*	Eric Cousineau - reflection (serialization?) changes

## Reflection (or just Serialization?)

This an attempt to generalize the structure of the URDF via reflection to make it easier to extend. This concept is taken from Gazebo's SDF structure, and was done with SDF in mind to a) make an SDF parser and b) make a simple converter between URDF and SDF.

### Changes

*	Features:
	*	Transmission and basic Gazebo nodes.
	*	General aggregate type, for preserving the order of aggregate types (i.e., the URDF robot model, and for the future, SDF models, links with multiple visuals / collisions, etc).  This allows for basic checks to see if "scalar" elements are defined multiple times. (We were affected by this at one point with a pose defined twice with different values, screwing up the loading / saving of a model with this API).
	*	Dumping to YAML, used for printing to string.
		*	Doesn't preserve ordering because of it, but someone posted an issue about this on pyyaml's issue tracker, and I think a few solutions were posted (including one from the author)
*	XML Parsing: minidom has been swapped out with lxml.etree, but it should not be hard to change that back. Maybe Sax could be used for event-driven parsing.
*	API:
	*	Loading methods rely primarily on instance methods rather than static methods, mirroring Gazebo's SDF construct-then-load method
	*	Renamed static `parse_xml()` to `from_xml()`, and renamed `load_*` methods to `from_*` if they are static

### Todo

1.	Develop a Python SDF API in a `sdf` module.
	*	Maybe make the package itself be `robot_model_py` so that the respective modules would be `robot_model_py.urdf_parser` and `robot_model_py.sdf_parser`?
	*	Parse Gazebo's SDF definition files at some point? For speed's sake, parse it and have it generate code to use?
2.	Make a direct, two-way URDF <-> SDF converter.
	*	Gazebo has the ability to load URDFs and save SDFs, but it lumps everything together and (I think) adds some "noise" from OpenDE for positions.
3.	Make the names a little clearer, especially the fact that `from_xml` and `to_xml` write to a node, but do not create a new one.
4.	Figure out good policy for handling default methods. If saving to XML, write out default values, or leave them out for brevity (and to leave it open for change)? Might be best to add that as an option.
5.	Find a lightweight package that can handle the reflection aspect more elegantly. Enthought traits? IPython's spinoff of traits?
