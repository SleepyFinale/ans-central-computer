# Fleet architecture

What runs where, and every piece of data that crosses between a robot and the central PC.

## Where software runs

```mermaid
flowchart LR
  subgraph robot ["Each robot Pi (namespace /<robot>, loopback-only DDS)"]
    bring["robot.launch.py<br/>base, lidar, URDF"]
    norm["laser_scan_normalizer<br/>(+ ultrasonic fusion)"]
    slam["slam_toolbox<br/>/<robot>/map, map->odom"]
    nav2["Nav2<br/>plans in <robot>/map"]
    wire["map_wire_compressed_republisher"]
    tfz["tf_zenoh_relay"]
    zr["zenoh-bridge-ros2dds (client)"]
    bring --> norm --> slam --> nav2
    slam --> wire --> zr
    bring --> tfz --> zr
    zr <--> nav2
  end
  subgraph central ["Central PC (ROS domain 50, localhost)"]
    zc["zenoh-bridge-ros2dds (router :7447)"]
    rx["fleet_zenoh_receive.py"]
    mm["map_merge (2+ robots)<br/>or map relay (1 robot)"]
    mon["map_merge_state_monitor"]
    ex["multi_robot_explorer"]
    rv["RViz"]
    zc --> rx
    rx -->|"/tf, /tf_static, /<robot>/map"| mm
    mm -->|"/map, map-><robot>/map"| ex
    mon --> ex
    ex <-->|"NavigateToPose,<br/>ComputePathToPose"| zc
    rx --> rv
    mm --> rv
  end
  zr <-->|"Tailscale TCP"| zc
```

A robot never receives TF or maps from the central PC. It only receives action goals, so it keeps its own consistent view and a dropped link stops new goals without confusing Nav2.

## Data on the link

Allowlists: `config/zenoh/central_router.json5.tmpl` here and `config/zenoh/robot_client.json5.tmpl` in the turtlebot3 workspace. Keep them in step.

| Robot sends | Rate cap | Central turns it into |
| --- | --- | --- |
| `/<robot>/tf_zenoh` (changed edges only, best effort) | 10 Hz | `/tf` |
| `/<robot>/tf_static_zenoh` | 1 Hz | `/tf_static` (one latched message for the fleet) |
| `/<robot>/map_wire_z` (zlib-compressed OccupancyGrid) | 1 Hz | `/<robot>/map` |
| `/<robot>/nav2_collision_ahead`, `nav2_retrace_active` | — | read by the explorer (robot nodes are off by default) |
| `/<robot>/plan`, `scan_normalized`, `local_costmap/costmap` | 2/2/1 Hz | RViz only; sent while someone subscribes |

Nav2 actions `/<robot>/navigate_to_pose` and `/<robot>/compute_path_to_pose` are bridged both ways.

Robot frames already carry the robot prefix (`inky/odom`, `inky/base_footprint`, `inky/map`), so every robot's TF lands in one `/tf` tree without renaming. The world frame `map` belongs to the central PC: `map_merge` publishes `map -> <robot>/map`, or with one robot `single_robot_world_tf_bridge.py` publishes it as identity.

### Why TF goes through `tf_zenoh_relay`

The Zenoh bridge subscribes with the publisher's QoS. A reliable reader on `/<robot>/tf` back-pressures slam_toolbox, `map -> odom` goes stale, and Nav2 reports false goal success. The helper (C++ `tf_zenoh_relay` in the robot's `turtlebot3_navigation2`; `scripts/comms/tf_zenoh_publish.py` is the fallback) reads best-effort and republishes on `*_zenoh` topics instead.

### DDS settings on the central PC

The bridge injects topics through Cyclone's built-in localhost profile (`ROS_LOCALHOST_ONLY=1`, no `CYCLONEDDS_URI`). Every central process uses the same profile via `fleet_use_zenoh_dds` in `scripts/env/fleet_common.bash`. RViz is the exception: it uses `config/cyclonedds/central_localhost.xml` to get past Cyclone's 10-participant default cap.

## Exploration loop

1. Frontiers: free cells next to unknown cells on the merged `/map` (or a robot's own map early on; `mode: auto` switches when `map_merge/merge_state` reports `MERGED` stably).
2. Assignment: each idle robot gets the unclaimed frontier with the best size-versus-distance score; robots are kept apart. Distance is travel distance around walls (Dijkstra on the map, `use_geodesic_distance`), size counts only up to `frontier_gain_cap_m`, and turning back costs a little (`heading_penalty_m`), so a robot finishes the room it is in instead of crossing the map for a large frontier behind a wall.
3. Precheck: `compute_path_to_pose` on the robot. Goals with no plan are blacklisted and the next frontier is tried, unless plans to several different goals fail with the robot in place (`boxed_in_*`): then the robot is boxed in, not the frontiers, so nothing is blacklisted and the explorer waits while the robot's stuck_detector clears its costmap and narrows its radius.
4. Dispatch: the goal is transformed into `<robot>/map` and sent as `NavigateToPose`. Nav2 on the robot plans and drives.
5. Watchdogs: stalls, aborts and repeated near-identical goals are cancelled and blacklisted. When no frontiers remain, robots return to their start pose.

Parameters: `src/m-explore-ros2/explore/config/multi_robot_explorer.yaml`, `src/m-explore-ros2/map_merge/config/multirobot_params_unknown_poses.yaml`.
