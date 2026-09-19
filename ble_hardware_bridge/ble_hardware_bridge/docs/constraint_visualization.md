# BLE control constraint visualization

Generate the joystick-coordinate constraint plot from the package configuration:

```bash
ros2 run ble_hardware_bridge plot_control_constraints.py \
  --output /tmp/ble_control_constraints.png
```

Generate the linear/angular velocity constraint plot:

```bash
ros2 run ble_hardware_bridge plot_control_constraints.py \
  --space velocity \
  --output /tmp/ble_velocity_constraints.png
```

Run directly from the source tree with an explicit configuration:

```bash
python3 scripts/plot_control_constraints.py \
  --config config/ble_hardware_bridge.yaml \
  --output docs/ble_control_constraints.png
```

The plot uses vehicle-coordinate names: X is forward with its positive direction
up, and Y is steering with its positive direction left. Green points are motion
commands emitted by `ControlProtocol::map_twist()`. The blue star is the
separately accepted stop command `(0, 0)`. The displayed view is cropped around
the executable region, while feasibility is still evaluated over `[-1, 1]^2`.
The joystick-space region is shared by every gear because lower gears are
defined as scaled versions of the `0005` calibration.

The executable motion region applies all of the following constraints:

- joystick coordinates are inside `[-1, 1]`;
- nonzero forward X is outside the configured deadzone;
- angular velocity remains inside its calibrated limit;
- both differential wheel velocities remain in their configured range;
- steering Y is produced by the piecewise-linear calibration table.

Y has no independent `0.3` deadzone after forward X is active. The hatched
`Y in [-0.1, 0)` strip reflects the flat calibration segment: zero differential
is mapped to `Y=0`, so the current inverse mapping never emits the rest of that
strip.

The plotted boundaries use:

```text
v = longitudinal_gain * X ^ longitudinal_exponent
d = piecewise_linear_calibration(Y)
omega = d / (wheel_separation / 2)
left_wheel = v - d
right_wheel = v + d
```

Gear selection uses requested physical linear velocity:

```text
v < gear_0001_selection_threshold                  -> 0001
gear_0001_selection_threshold <= v
  < gear_0003_selection_threshold                  -> 0003
gear_0003_selection_threshold <= v                 -> 0005

gear_scale = selected_gear_maximum_linear_velocity
  / gear_0005_maximum_linear_velocity
equivalent_0005_v = v / gear_scale
equivalent_0005_omega = omega / gear_scale
```

The equivalent `0005` velocities are checked and converted to X/Y with the
reference calibration. Values between the longitudinal fit endpoint and the
configured `0005` maximum saturate X at `1.0`. This scales linear and angular
velocity together. The resulting BLE payload remains steering first, forward
second, then the selected gear as `00 01`, `00 03`, or `00 05`.

In the velocity plot, linear velocity is vertical and positive up. Angular
velocity is horizontal and positive left. The green region directly shows which
`(v, omega)` commands from `/cmd_vel` pass the BLE mapping constraints. Its lower
panel shows the three gear-scaled piecewise-linear inverse lookups from physical
angular velocity to encoded steering Y. The open point at
`(omega=0, Y=-0.1)` is the unused endpoint of the
flat zero-differential calibration segment; the implementation explicitly
selects the filled `(omega=0, Y=0)` point. Constraint intersections with the
coordinate axes are marked with their numeric values. Wheel-maximum intercepts
outside the displayed angular range are listed beside the main plot. Pairwise
constraint intersections inside the displayed ranges are marked with diamond
symbols and numeric coordinate pairs. The joystick plot uses `(Y, X)` and the
velocity plot uses `(omega, v)` for those labels.
