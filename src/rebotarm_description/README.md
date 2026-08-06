# rebotarm_description

Robot geometry and visualization resources for the reBotArm DM and RS models.

## Contents

- `urdf/dm`: DM URDF, Xacro wrapper, and meshes
- `urdf/rs`: RS URDF, Xacro wrapper, and meshes
- `launch/display.launch.py`: model visualization with joint controls
- `rviz/display.rviz`: default RViz layout

## Build

```bash
colcon build --packages-select rebotarm_description
source install/setup.bash
```

## Visualize

```bash
ros2 launch rebotarm_description display.launch.py model:=dm
ros2 launch rebotarm_description display.launch.py model:=rs
```

## Model data status

The mass, inertia, joint limit, effort, and velocity values currently come from
the source CAD exports. Verify them against the mechanical and actuator
specifications before using the models for dynamics, safety limits, or hardware
control. In particular, values such as joint velocities of `40` or `50` rad/s
and gripper effort of `500` should be treated as unverified.

The collision geometry currently reuses detailed visual meshes. Simplified
collision meshes are recommended before motion-planning or simulation workloads.
