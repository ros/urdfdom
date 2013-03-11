# robot_model_py

## Authors

*	Thomas Moulard
*	David Lu
*	Kelsey Hawkins
*	Antonio El Khoury
*	Eric Cousineau

## Description

A simple URDF parser for Python.

## Reflection

This is (unfortunately) a very different API. It is an attempt to generalize the structure of the URDF via reflection to make it easier to extend. This concept is taken from Gazebo's SDF structure.

This was done with SDF in mind to a) make an SDF parser and b) make a simple converter between URDF and SDF.

## Todo

1.	Make an SDF parser (some preliminary work is done on this, but not published) in a `sdf` module.
	*	Maybe make the package itself be `robot_model_py` so that the respective modules would be `robot_model_py.urdf_parser` and `robot_model_py.sdf_parser`?
2.	Make an URDF <-> SDF converter.
3.	Make the names a little clearer, especially the fact that `from_xml` and `to_xml` write to a node, but do not create a new one.
4.	Make the URDF parsing consistent with the rest of the structure, i.e., with `required='*'`.
