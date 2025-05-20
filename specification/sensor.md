# `<sensor>` element


The sensor element describes basic properties of a visual sensor (i.e. camera / ray sensor).


## Attributes

| attribute     | type     | use      | default value                                                                                                                | description                 |
| ------------- | -------- | -------- | ---------------------------------------------------------------------------------------------------------------------------- | --------------------------- |
| `name`        | `string` | required | NA                                                                                                                           | The name of the sensor link |
| `update_rate` | `double` | optional | The frequency at which the sensor data is generated in `hz`. If left unspecified, the sensor will generate data every cycle. |

## Elements

| element                | use      | description                                                                                  |
| ---------------------- | -------- | -------------------------------------------------------------------------------------------- |
| [`<parent>`](#parent)   | required | Defines the parent link this sensor is attached to.                                          |
| [`<origin>`](#origin) | optional | This is the pose of the sensor optical frame, relative to the sensor parent reference frame. |
| [`<camera>`](#camera) | optional | Camera sensor definition.                                                                    |
| [`<ray>`](#ray)        | optional | Laser sensor definition.                                                                     |

### `<parent>`


| attribute | type     | use      | default value | description                                             |
| --------- | -------- | -------- | ------------- | ------------------------------------------------------- |
| `link`    | `string` | required | NA            | The name of the parent link this sensor is attached to. |

### `<origin>`

| attribute     | type     | use      | default value                                                                                                                | description                 |
| --------- | -------- | -------- | ------------- | ------------------------------------------------------- |
| `xyz` | `string` | optional | zero vector | Represents the offset with respect to the parent frame. |
| `rpy` | `string` | optional | zero vector | Represents the fixed axis roll, pitch and yaw angles in radians. |

### `<camera>`

The `<camera>` element has following child elements:
| element                    | use      | description                   |
| -------------------------- | -------- | ----------------------------- |
| [`<image>`](#camera-image) | required | Defines the image parameters. |

#### camera: `<image>`

| attribute | type           | use      | default value | description                                                                                                                                                                                        |
| --------- | -------------- | -------- | ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `width`   | `unsigned int` | required | NA            | Width of the camera in `pixels`.                                                                                                                                                                   |
| `height`  | `unsigned int` | required | NA            | Height of the camera in `pixels`.                                                                                                                                                                  |
| `format`  | `string`       | required | NA            | Can be any of the strings defined in [image_encodings.h inside sensor_msgs](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/include/sensor_msgs/image_encodings.h). |
| `hfov`    | `double`       | required | NA            | Horizontal field of view of the camera in `radians`.                                                                                                                                               |
| `near`    | `double`       | required | NA            | Near clip distance of the camera in `meters`.                                                                                                                                                      |
| `far`     | `double`       | required | NA            | Far clip distance of the camera in `meters`. This needs to be greater or equal to near clip.                                                                                                       |

### `<ray>`

The `<ray>` element has following child elements:
| element        | use      | description                                     |
| -------------- | -------- | ----------------------------------------------- |
| `<horizontal>` | optional | Defines the horizontal parameters of the laser. |
| `<vertical>`   | optional | Defines the vertical parameters of the laser.   |

#### ray: `<horizontal>`

| attribute    | type           | use      | default value                                                       | description                                                                                                                                                                                                        |
| ------------ | -------------- | -------- | ------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `samples`    | `unsigned int` | optional | 1                                                                   | The number of simulated rays to generate per complete laser sweep cycle.                                                                                                                                           |
| `resolution` | `unsigned int` | optional | 1                                                                   | This number is multiplied by samples to determine the number of range data points returned. If resolution is less than one, range data is interpolated. If resolution is greater than one, range data is averaged. |
| `min_angle`  | `double`       | optional | Minimun angle in `radians`.                                         |
| `max_angle`  | `double`       | optional | Maximum angle in `radian`. Must be greater or equal to `min_angle`. |

#### ray: `<vertical>`

| attribute    | type           | use      | default value                                                       | description                                                                                                                                                                                                        |
| ------------ | -------------- | -------- | ------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `samples`    | `unsigned int` | optional | 1                                                                   | The number of simulated rays to generate per complete laser sweep cycle.                                                                                                                                           |
| `resolution` | `unsigned int` | optional | 1                                                                   | This number is multiplied by samples to determine the number of range data points returned. If resolution is less than one, range data is interpolated. If resolution is greater than one, range data is averaged. |
| `min_angle`  | `double`       | optional | Minimun angle in `radians`.                                         |
| `max_angle`  | `double`       | optional | Maximum angle in `radian`. Must be greater or equal to `min_angle`. |

## Recommended Camera or Ray Resolution

In simulation, large sensors will slow down overall performance. Depending on update rates required, it is recommended to keep the camera or ray resolution and update rates as low as possible.

## Example

Here is an example of a camera sensor element:
```xml
<sensor name="my_camera_sensor" update_rate="20">
    <parent link="optical_frame_link_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <camera>
        <image width="640" height="480" hfov="1.5708" format="RGB8" near="0.01" far="50.0"/>
    </camera>
</sensor>
```

And below is an example of a laser scan (ray) sensor element:

```xml
<sensor name="my_ray_sensor" update_rate="20">
    <parent link="optical_frame_link_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <ray>
        <horizontal samples="100" resolution="1" min_angle="-1.5708" max_angle="1.5708"/>
        <vertical samples="1" resolution="1" min_angle="0" max_angle="0"/>
    </ray>
</sensor>
```
