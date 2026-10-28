# my_nav_control

Shared control-layer nodes for the custom navigation stack.

- `path_follower_node`: consumes `/planned_path`, uses TF to track the robot in `map`, and publishes `/cmd_vel`
- `fake_base_driver_node`: simulates an `odom -> base_link` backend for development without hardware
