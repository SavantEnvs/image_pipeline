Plain-struct stand-ins for the rosidl-generated message headers (`sensor_msgs`, `std_msgs`,
`builtin_interfaces`), carrying exactly the fields of the ROS 2 `.msg` definitions. The generated
headers cannot be produced here without a ROS 2 install (Debian ships only ROS 1). Everything that
operates on these types is real: `sensor_msgs/point_cloud2_iterator.hpp`, `image_encodings.hpp` and
`image_geometry` come from the pinned upstream sources in `/opt/toolchains/deps/`.
