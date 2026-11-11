advrf_ros2_gravity_compensation
=================================

Launch the gravity compensation: 
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
