# `<robot>` element

The root element in a robot description file must be a `<robot>` element. All other elements must be encapsulated within it.

## Attributes

| `<robot>` attr | type   | use      | default value | description                                                                                                                                                                                                               |
| -------------- | ------ | -------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `name`         | string | required | NA            | The `robot` element must have a *name* attribute. |
| `version`      | string | optional | "1.0"         | version of the urdf specification in `<major>.<minor>` format.                                                                                                                                                            |

## Elements

| `<robot>` element                     | description                                                          |
| ------------------------------------- | -------------------------------------------------------------------- |
| [`<link>`](./link.md)                 | defines a link with its own frame.                                   |
| [`<joint>`](./joint.md)               | mandatory joint frame definition.                                    |
| [`<transmission>`](./transmission.md) | (PR2 specific).                                                      |
| [`<gazebo>`](./gazebo.md)             | [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs) simulation extensions. |

## Example

```xml
<robot name="pr2">
  <!-- pr2 robot links and joints and more -->
</robot>
```
