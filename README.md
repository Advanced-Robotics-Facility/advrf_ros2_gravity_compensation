# advrf_ros2_gravity_compensation
ros version: jazzy

## Quick Use

```bash
ros2 launch advrf_ros2_miranda_bringup gravity_compensation.launch.py start_bridge:=true active_tx_on_first_msg:=false use_rviz:=true
```

Check topic with:
```bash
ros2 topic echo /advrf/miranda/ecat/tx/motors
```

Enable the EtherCAT transmission:
```bash
ros2 service call /advrf/miranda/ecat/set_active_tx std_srvs/srv/SetBool "{data: true}"
```

Enable the gravity compensation:
```bash
ros2 service call /advrf/miranda/set_gravity_compensation std_srvs/srv/SetBool "{data: true}"
```

Disable the gravity compensation:
```bash
ros2 service call /advrf/miranda/set_gravity_compensation std_srvs/srv/SetBool "{data: false}"
```

Disable the EtherCAT transmission:
```bash
ros2 service call /advrf/miranda/ecat/set_active_tx std_srvs/srv/SetBool "{data: false}"
```

## Edit Config

The generic launch lives in `advrf_ros2_bringup`. The Miranda launch supplies
robot resources and defaults to `advrf_ros2_miranda_bringup/config/gravity_compensation.yaml`.
Edit its `/**/gravity_compensation.ros__parameters.joints` list to select the joints.
Motor IDs and impedance gains are matched by name from the robot configuration.
An empty list, duplicates or unknown names are rejected before starting the node.

Use another file with:
```bash
ros2 launch advrf_ros2_miranda_bringup gravity_compensation.launch.py gravity_config:=/path/to/gravity_compensation.yaml
```

In the generic launch, omitting `gravity_config` selects all configured joints.

| Parameter | Purpose |
| --- | --- |
| `robot_description` | Nonempty URDF used by the gravity model |
| `joints` | Unique controlled joint names |
| `motor_ids` | Unique EtherCAT IDs, one per joint |
| `joint_gains` | Five finite gain values per joint |
| `gain_ramp_duration` | Positive gain transition duration, default `3.0` seconds |

## Nodes Details

Node: `gravity_compensation` (executable `advrf_ros2_gravity_compensation`).

- Subscribe: `rx/joint_states` (`sensor_msgs/msg/JointState`).
- Publish: `tx/motors` (`advrf_ros2_interfaces/msg/MotorTxPdoVector`).
- Service: `set_gravity_compensation` (`std_srvs/srv/SetBool`).
- Enabling ramps position stiffness and damping to zero while applying gravity torque; disabling restores configured gains and returns to hold.
- Commands are updated on valid joint feedback.

## Dependencies

- `rclcpp`
- `sensor_msgs`
- `std_srvs`
- `advrf_ros2_interfaces`

## TroubleShooting

| Symptom | Check |
| --- | --- |
| Startup rejects configuration | URDF, unique joint names/IDs and five gains per joint |
| No commands | Feedback topic, namespace and positions for every controlled joint |
| No robot response | Bridge TX activation and EtherCAT backend |
| Model creation fails | XBot model backend and robot description |
