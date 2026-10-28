# my_nav_bringup

Bringup package for the custom navigation stack with RMU Gazebo simulation.

## Run

See the root [startup guide](../../STARTUP_GUIDE.md) for full setup and troubleshooting.

```bash
ros2 launch my_nav_bringup sim.launch.py
ros2 launch my_nav_bringup reality.launch.py
```

### Main topics

- `/map`
- `/odom`
- `/goal_pose`
- `/planned_path`
- `/cmd_vel`

### Modes

- `sim.launch.py` starts `rmu_gazebo_simulator` plus the shared planner/control stack
- `reality.launch.py` starts the same planner/control stack without Gazebo

The simulation path targets `red_standard_robot1` by default and bridges `/cmd_vel`
to `/<robot_name>/robot_base/chassis_cmd` when the RMU message package is available.

### Demo goal

The planner uses a default goal of `(2.8, 2.8)` unless `/goal_pose` is published.
