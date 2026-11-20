# Bring-up checklist

Bring the system up one layer at a time. Each step runs on its own and tells you clearly when it
works, so a problem shows up where it is cheapest to fix. Once everything has worked once, the
pre-flight checklist plus steps 1 and 4 are enough before each session.

Every terminal needs:

```bash
source /opt/ros/humble/setup.bash
source /etc/turtlebot4_discovery/setup.bash   # discovery server settings, see hardware_setup.md
source ~/ros2_ws/install/setup.bash
```

## Pre-flight checklist

- [ ] TurtleBots on and undocked; Crazyflies **off**.
- [ ] Batteries charged: Crazyflies above `min_takeoff_voltage` (3.8 V), TurtleBots above ~20 %.
- [ ] Both Crazyradios plugged in: `lsusb | grep -i bitcraze`.
- [ ] Motive tracks every rigid body, and the MoCap driver is running (step 1).
- [ ] Arena clear, nobody inside.
- [ ] Each Crazyflie on its UGV's deck, facing the UGV's front, each UGV near its team's start point.
- [ ] Power the Crazyflies on (wait 10 s); power-cycle them after every run.

## 1. MoCap

Start your MoCap driver (for `mocap_optitrack`: `ros2 launch mocap_optitrack mocap.launch.py`),
then check every rigid body:

```bash
ros2 topic hz /Robot_1/pose         # steady rate, e.g. 120 Hz
ros2 topic echo --once /Robot_1/pose
```

Push a robot along the arena's +x: `position.x` should increase. A robot on the floor reads
`z ≈ 0`, a Crazyflie on its deck reads `z ≈ deck_height`.

## 2. TurtleBots are reachable

```bash
ros2 topic list | grep -E 'ugv[0-9]+/(scan|cmd_vel)'
```

Nothing listed? See *discovery* in [troubleshooting](troubleshooting.md).

## 3. LiDAR orientation

```bash
ros2 run uav_ugv_nav scan_check --ros-args -r __ns:=/ugv1
```

Hold a box about 0.5 m **in front** of the robot: the closest return should read about `0 deg`
and `ahead`. On the robot's left it should read about `+90 deg`. Otherwise fix
`lidar_yaw_offset_deg` in `params.yaml` and pass it here with `-p lidar_yaw_offset_deg:=...`.
Repeat for every UGV.

## 4. Crazyflies on the ground (motors stay off)

```bash
ros2 run uav_ugv_nav cf_check --ros-args -p uri:=radio://0/80/2M/E7E7E7E7E7 -p mocap_topic:=/Robot_3/pose
```

It connects, selects the Kalman filter, streams MoCap, resets the filter and must end with
`PASS: estimate within 0.0xx m of MoCap`. Repeat for every Crazyflie, then power-cycle them.

## 5. The mission

```bash
ros2 run uav_ugv_nav check_mission my_mission.yaml --fleet my_fleet.yaml
```

Every tour must say `ok`, and every point must be inside the arena.

## 6. UGVs alone

With the Crazyflies off (or not on the decks):

```bash
ros2 launch uav_ugv_nav mission.launch.py mission:=my_mission.yaml fleet:=my_fleet.yaml \
    robots:=ugv standalone:=true
```

Each UGV drives to every release and collect point of its tours, pausing at each, then parks at
its end point. Put an obstacle on a path to see it swerve. Ctrl+C stops everything.

## 7. UAVs alone

One team at a time. Put the Crazyflie **on the floor** in a clear spot near its tour, facing +x:

```bash
ros2 launch uav_ugv_nav mission.launch.py mission:=my_mission.yaml fleet:=my_fleet.yaml \
    robots:=uav standalone:=true teams:=team1
```

The UAV runs its pre-flight check, takes off, flies the waypoints of each tour and lands back
where it took off after each one. Ctrl+C makes it land where it is.

## 8. One team, then all teams

Crazyflie on its UGV's deck facing forward, UGV at the team's start point:

```bash
ros2 launch uav_ugv_nav mission.launch.py mission:=my_mission.yaml fleet:=my_fleet.yaml teams:=team1
```

Then without `teams:=` for all teams. Watch the per-tour reports:

```
[cf2.uav_node]: tour 2/2 done: flight 32.9 s (model 31.3 s), air slack 57.1 s (ok, min 20 s); ...
[ugv2.ugv_node]: tour 2/2: done. Release->collect drive 23.8 s, ground slack 66.2 s (min 20 s); ...
```

If the realized flight times run well above the model (arming, filter settling and radio
latency are not in the model), raise `max_flight_time` or the model durations in the mission
so the planned slack reflects reality.
