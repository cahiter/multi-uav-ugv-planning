# Mission format

Two YAML files describe a run:

- the **mission** says what each team does: a timing model and, per team, a start point, an end
  point and a list of tours. Planners write this file.
- the **fleet** says which robots form each team, how to reach them, and the arena's safe box.
  It belongs to the lab and rarely changes.

Team names (`team1`, `team2`, …) connect the two. Coordinates are in the MoCap world frame, in
meters, z up.

## Mission

```yaml
model:                    # optional; every key has the default shown
  max_flight_time: 90.0   # flight-time budget per tour [s]
  min_air_slack: 20.0     # each tour's flight must leave at least this much budget unused [s]
  min_ground_slack: 20.0  # same for the UGV's release->collect drive [s]
  flight_altitude: 0.8    # altitude of waypoints given as [x, y] [m]
  uav_speed: 0.3          # UAV speed between waypoints [m/s]
  min_leg_time: 2.0       # shortest single UAV move [s]
  hover_time: 3.0         # hover at each waypoint [s]
  takeoff_time: 3.0       # [s]
  settle_time: 2.0        # align above the UGV before descending [s]
  land_time: 4.0          # [s]
  ugv_speed: 0.12         # average UGV speed including turns [m/s]
  recharge_time: 0.0      # minimum time on the deck between tours [s]

teams:
  team1:
    start: [-3.6, 0.5]    # where the UGV starts, carrying its UAV
    end: [-3.8, 4.1]      # where the UGV parks after the last tour
    tours:
      - release: [-3.5, 1.1]                                    # the UGV drops the UAV here
        waypoints: [[-4.0, 0.9], [-2.7, 1.6]]                   # [x, y] or [x, y, z]
        collect: [-3.3, 1.8]                                    # the UGV picks the UAV up here
      - release: [-3.3, 2.3]
        waypoints: [[-2.7, 2.4], [-3.3, 2.7, 1.1], [-3.9, 2.9]]
        collect: [-3.3, 3.0]
      - ...
  team2:
    ...
```

Teams may have different numbers of tours, and tours different numbers of waypoints, but every
tour needs at least one waypoint. Unknown keys are rejected, so a typo like `waypionts` fails
loudly instead of being ignored.

### Timing model

The UAV flies with exactly these durations, so the model and the robots agree:

- A UAV move from `a` to `b` takes `max(min_leg_time, |b − a| / uav_speed)`.
- A tour's **flight time** is `takeoff_time`, plus the moves release → waypoints → collect at
  altitude, plus `hover_time` at every waypoint, plus `settle_time` and `land_time`.
- A UGV drive from `a` to `b` takes `|b − a| / ugv_speed` (straight line).
- **Air slack** is `max_flight_time` minus the flight time; **ground slack** is `max_flight_time`
  minus the UGV's release → collect drive. A tour is acceptable when they reach `min_air_slack`
  and `min_ground_slack`. That margin is what absorbs surprises during the run: a UGV detouring
  around an obstacle, a UAV hovering while it waits for its UGV, a collect point moved a little.
- A team's **mission time** is its drive to the first release point, the slower of flight and
  drive for every tour, the longer of drive and `recharge_time` between tours, and the drive to
  the end point.

During a run the robots log the *realized* flight time, drive time and slack of every tour next
to these estimates.

## Fleet

```yaml
arena:                    # optional safe box; every mission point must lie inside
  x: [-4.5, 0.6]
  y: [0.1, 4.6]
  z: [0.0, 1.5]

teams:
  team1:
    uav: {name: cf1, uri: radio://0/80/2M/E7E7E7E7E7, mocap_topic: /Robot_3/pose}
    ugv: {name: ugv1, mocap_topic: /Robot_1/pose}
```

`name` is the ROS namespace of the robot's node (`/cf1/uav_node`, `/ugv1/ugv_node`); for a
TurtleBot it must be the robot's own namespace. Names, MoCap topics and radio URIs must be
unique across the fleet.

## Checking a mission

```bash
ros2 run uav_ugv_nav check_mission my_mission.yaml --fleet my_fleet.yaml
```

```
team1: 3 tours, 7 waypoints
  tour  waypoints  UAV flight  air slack  UGV drive  ground slack
     1          2      24.0 s     66.0 s      6.1 s        83.9 s  ok
     2          3      26.8 s     63.2 s      5.8 s        84.2 s  ok
     3          2      23.6 s     66.4 s      5.1 s        84.9 s  ok
  estimated mission time: 91.4 s
...
OK: every tour leaves the required slack, and every point is inside the arena.
```

It exits with status 1 if a tour leaves too little slack, a point is outside the arena or a
mission team has no robots, so you can use it in scripts. `mission.launch.py` runs the same
checks: it refuses to start on arena or fleet problems and warns about slack violations.

## Writing a mission from a planner

A mission is plain data, so any planner can produce one:

```python
import yaml

plan = {
    'model': {'max_flight_time': 90.0, 'min_air_slack': 20.0, 'min_ground_slack': 20.0},
    'teams': {
        f'team{k + 1}': {
            'start': list(start), 'end': list(end),
            'tours': [{'release': list(r), 'waypoints': [list(p) for p in pts], 'collect': list(c)}
                      for r, pts, c in tours],
        }
        for k, (start, end, tours) in enumerate(my_planner_output)
    },
}
with open('my_mission.yaml', 'w') as f:
    yaml.safe_dump(plan, f, sort_keys=False)
```

`uav_ugv_nav.mission` also loads and validates missions from Python (`load_mission`,
`uav_tour_time`, `tour_slack`, `constraint_violations`). Planners can use it to check their output
against the same model.
