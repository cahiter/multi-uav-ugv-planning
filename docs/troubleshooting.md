# Troubleshooting

Problems we ran into, roughly in the order you meet them during [bring-up](bringup.md).

## Network and MoCap

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros2 topic list` shows no TurtleBot topics | Discovery server settings missing, or servers listed in the wrong order | Source the discovery setup in every terminal, list the servers in the order of their IDs, set `ROS_SUPER_CLIENT=True`, restart the daemon (`ros2 daemon stop; ros2 daemon start`). On each robot, check that the server runs: `ps aux \| grep fastdds` |
| `no fresh MoCap pose on /Robot_N/pose; holding still` | Rigid body not tracked, driver not running, or wrong topic in `fleet.yaml` | `ros2 topic hz /Robot_N/pose`; check the rigid body in Motive |
| Heights look wrong, or a robot on the floor has z far from 0 | Motive streams y-up | Set the *Up Axis* to *Z* in Motive's streaming settings |

## Crazyflie

| Symptom | Likely cause | Fix |
|---|---|---|
| `cannot connect to radio://...: Cannot find a Crazyradio Dongle` | Crazyradio unplugged, no USB permissions, or the URI's radio index is wrong | `lsusb \| grep -i bitcraze`; set up the udev rules (see [hardware setup](hardware_setup.md)) |
| `cannot connect ... Too many packets lost` | Crazyflie off, wrong channel or address, or two Crazyflies on one channel | Check the channel and address in `cfclient`; give every Crazyflie its own channel |
| `could not select the Kalman filter` | Parameters sent before the firmware was ready | Power-cycle; `Crazyflie.connect()` already waits 3 s, raise `startup_delay` if needed |
| Pre-flight check: estimate meters away from MoCap | With firmware 2026.04 we saw this with full-pose updates (`send_extpose`); or the MoCap topic belongs to another robot | Keep position-only `send_extpos` (as in this code); check `mocap_topic` in `fleet.yaml` |
| Pre-flight check: `estimate is stuck at the origin` | No MoCap positions reach the Crazyflie, or corrupted firmware (`kalman.varPX` stays exactly 0) | Check the MoCap topic; otherwise do a cold-boot recovery in `cfclient` and reflash the firmware, then set the channel and address again |
| Log callbacks never arrive | A log block is limited to 26 bytes (6 floats) | Split the variables over several `LogConfig`s |
| Drifts or spirals right after takeoff | The Crazyflie did not start facing world +x (position-only MoCap gives the filter no yaw) | Face it to the UGV's front on the deck; keep `align_yaw: true` |
| Lands beside the deck | UGV rigid-body pivot not at the deck center | Move the pivot in Motive |
| Hard landing, or motors stop above the deck | Wrong `deck_height` | Measure the Crazyflie's MoCap z on the deck; the UAV node warns at the first takeoff if they differ by more than 5 cm |
| `ABORT (MoCap pose lost ...)` | Occlusion longer than `mocap_timeout` | Check marker visibility at low altitude above the deck; raise `mocap_timeout` slightly if the occlusions are brief |
| Realized flight times well above the model | Arming, filter settling and radio latency are not in the model | Give the mission more `max_flight_time`, or longer model durations |

## TurtleBot 4

| Symptom | Likely cause | Fix |
|---|---|---|
| `no LiDAR scan on /ugvN/scan; holding still` | Wrong namespace, or the LiDAR is not running | `ros2 topic hz /ugvN/scan`; set `require_scan: false` only to test without a LiDAR |
| Ignores obstacles ahead, but swerves at things beside it | Wrong `lidar_yaw_offset_deg` | Check with `scan_check` (bring-up step 3) |
| Oscillates or stops and goes near obstacles | APF gain too high, or the robot sees itself | Lower `apf_gain` (0.03–0.08); keep `lidar_min_range` at 0.15 m |
| `stuck: turning in place to escape`, again and again | APF local minimum: obstacles that form a pocket, or a point inside clutter | Raise `apf_vortex`; keep release and collect points away from clutter. A reactive APF cannot plan around large obstacles |
| `emergency stop: obstacle ... ahead` | Something within `stop_distance` in the forward arc | Expected; it drives on once the way is clear |
| `waiting for /cf1/drone_takeoff (is the uav_node running?)` | The team's `uav_node` is not running or crashed | Start it (or its launch); the UGV waits at the release point |
| `cf1 did not take off: ...` and the mission stops | The UAV refused or failed the pre-flight check; the reason is in the message | Fix the reason, power-cycle the Crazyflie, restart the team's nodes together |

## Software

| Symptom | Likely cause | Fix |
|---|---|---|
| `numpy.core.multiarray failed to import` from system Python packages after `pip install cflib` | pip installed numpy 2.x into `~/.local`, while Humble's apt packages are built for numpy 1.x | `pip install --user "numpy<2"` |
