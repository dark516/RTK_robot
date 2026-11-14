# rover_lidar_filter

ROS 2 node that removes lidar returns located inside the rover chassis footprint.
The scan points are transformed into `base_link`, so the lidar's forward offset and
its yaw are handled through TF rather than by hard-coded angular masks.

Default topics:

- input: `/scan` (raw driver output)
- output: `/scan_filtered` (used by SLAM, Nav2, RViz and the rover agent)

The filtered samples keep their original array indices and are replaced with
positive infinity, preserving the `sensor_msgs/msg/LaserScan` geometry.
