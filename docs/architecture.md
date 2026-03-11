# Capstone architecture (nested “boxes in boxes” diagrams)

These diagrams are meant to answer, at a glance:

- **What** big functions exist (outer box)
- **Which ROS2 programs/nodes** implement them (box inside)
- **Which ROS2 data flows** (topics/TF/actions) connect them (innermost box)

If you keep the diagrams “small and many” (instead of “one giant”), they stay readable and are easy to update as your capstone evolves.

---

## Legend + reusable template

- **Outer box** = Capstone function (capability)
- **Middle box** = ROS2 programs (nodes, launch files, scripts)
- **Inner box** = Data contracts (topics, TF frames, actions, services)
- **Arrow direction** = producer → consumer
- **Arrow label** = the specific contract (e.g., `topic:/map`, `tf:map->odom`, `action:NavigateToPose`)

```mermaid
flowchart TB
  subgraph FunctionExample["Function: SomeCapability"]
    direction TB

    subgraph Programs["Programs (ROS2 nodes/scripts/launch)"]
      direction TB
      progA["node: some_node"]
      progB["launch: some_stack.launch.py"]
    end

    subgraph Data["Data (topics / TF / actions)"]
      direction TB
      t1["topic:/some_topic (Type)"]
      tf1["tf: frameA -> frameB"]
      act1["action:/some_action (ActionType)"]
    end

    progA -->|"topic:/some_topic"| progB
    progB -->|"action:/some_action"| progA
    progA -->|"tf: frameA->frameB"| tf1
  end
```

---

## System context (Central ↔ Robots ↔ GUI)

```mermaid
flowchart TB
  subgraph Robots["Individual Robots (SBCs)"]
    direction TB
    r1["Robot: Blinky (ns=/blinky, ROS_DOMAIN_ID=50)"]
    r2["Robot: Pinky (ns=/pinky, ROS_DOMAIN_ID=50)"]
    rN["Robot: Inky/Clyde/... (ns=/inky,/clyde, ROS_DOMAIN_ID=50)"]
  end

  subgraph Central["Central Computer (Shared Domain: ROS_DOMAIN_ID=50)"]
    direction TB
    centralMapping["Function: GlobalMapping"]
    centralTasking["Function: Frontier+TaskAllocation"]
    centralSecurity["Function: SecurityMonitoring"]
  end

  subgraph GUI["Graphical User Interface"]
    direction TB
    guiMap["Global map + robot poses"]
    guiObjects["Object markers"]
    guiVideo["Live video feeds"]
    guiStatus["Robot status panel"]
  end

  Robots -->|"sensor+pose+maps (namespaced topics, shared domain)"| Central
  Central -->|"waypoints / safety commands (capstone contract)"| Robots
  Central -->|"map+poses+events"| GUI
  Robots -->|"video + object detections"| GUI
```

---

## Function: GlobalMapping (multi-robot SLAM + map merge) — matches current workspace

This function already exists in your central workspace. Robots (Blinky, Pinky, Inky, etc.) now run **namespaced** Nav2 + SLAM stacks on a shared domain (e.g. `ROS_DOMAIN_ID=50`), and the central computer consumes their maps and TF directly without domain bridges.

- **TF stitching**: `scripts/tf_relay_multirobot.py` (merges `/blinky/tf`, `/pinky/tf`, `/inky/tf` into `/tf` with prefixes)
- **Per-robot SLAM**: `slam_toolbox` instances on each robot, publishing `/blinky/map`, `/pinky/map`, etc.
- **Map merge**: `multirobot_map_merge/map_merge` using `config/map_merge/multirobot_params_unknown_poses.yaml` (for unknown initial poses on the central computer)

```mermaid
flowchart TB
  subgraph Function_GlobalMapping["Function: GlobalMapping (LocalMaps -> GlobalMap)"]
    direction TB

    subgraph Programs_GlobalMapping["Programs"]
      direction TB
      tfRelay["script: scripts/tf_relay_multirobot.py"]
      slamB["node: slam_toolbox (Blinky on robot)\nasync_slam_toolbox_node\n(robot-side params in ans-turtlebot3 workspace)"]
      slamP["node: slam_toolbox (Pinky on robot)\nasync_slam_toolbox_node\n(robot-side params in ans-turtlebot3 workspace)"]
      mapMerge["node: multirobot_map_merge/map_merge\nconfig/map_merge/multirobot_params_unknown_poses.yaml"]
    end

    subgraph Data_GlobalMapping["Data (topics + TF)"]
      direction TB
      bMap["topic:/blinky/map (nav_msgs/OccupancyGrid)"]
      pMap["topic:/pinky/map (nav_msgs/OccupancyGrid)"]
      gMap["topic:/map (nav_msgs/OccupancyGrid)"]

      bTF["topic:/blinky/tf + /blinky/tf_static (tf2_msgs/TFMessage)"]
      pTF["topic:/pinky/tf + /pinky/tf_static (tf2_msgs/TFMessage)"]
      gTF["topic:/tf + /tf_static (tf2_msgs/TFMessage)"]

      tfChain["TF chains (intended):\nmap -> blinky/map -> blinky/odom -> blinky/base_footprint\nmap -> pinky/map -> pinky/odom -> pinky/base_footprint"]
    end

    bTF -->|"script: prefix+merge"| tfRelay
    pTF -->|"script: prefix+merge"| tfRelay
    tfRelay -->|"topic:/tf + /tf_static"| gTF

    slamB -->|"topic:/blinky/map"| bMap
    slamP -->|"topic:/pinky/map"| pMap

    bMap -->|"map_merge inputs"| mapMerge
    pMap -->|"map_merge inputs"| mapMerge
    mapMerge -->|"topic:/map"| gMap
    mapMerge -->|"tf: map-><robot>/map (publish_tf)"| tfChain
  end
```

### Note on laser scan normalization

- Your bridged topic list currently includes **`/blinky/scan_normalized`** and **`/pinky/scan_normalized`** (see `config/domain_bridge/*_bridge.yaml`), and your SLAM params consume `scan_topic: /<robot>/scan_normalized`.
- The `multirobot_slam.launch.py` file also starts a central normalizer that expects `/<robot>/scan`. If you keep using robot-side normalization (common), that central normalizer is effectively optional. If you want central-side normalization, ensure `/<robot>/scan` is bridged into domain 50.

---

## Function: Frontier detection + waypoint assignment + navigation (capstone contract)

Your current workspace includes per-robot **Nav2** and per-robot **Explore Lite** in the aggregation domain:

- Launch: `src/turtlebot3/turtlebot3_navigation2/launch/multirobot_nav2_explore.launch.py`
- Starter: `scripts/start_multirobot_nav2_explore.sh`

For the capstone requirement (“optimized waypoints”, “minimize overlap”), you typically add a **central TaskAllocator** that assigns frontiers/waypoints per robot (instead of each robot greedily exploring on its own).

```mermaid
flowchart TB
  subgraph Function_TaskAndNav["Function: Frontier+TaskAllocation+Navigation"]
    direction TB

    subgraph Programs_TaskAndNav["Programs"]
      direction TB
      globalMapSrc["Global map provider\n(map_merge from GlobalMapping)"]

      frontierDetect["node: FrontierDetector\n(current: explore_lite per-robot)\n(capstone: central frontier service)"]
      taskAllocator["node: TaskAllocator (central)\n(assigns non-overlapping goals)"]

      nav2B["stack: Nav2 (namespace: blinky)\nplanner/controller/BT navigator"]
      nav2P["stack: Nav2 (namespace: pinky)\nplanner/controller/BT navigator"]

      explorerB["node: explore_lite (blinky namespace)\n(current behavior)"]
      explorerP["node: explore_lite (pinky namespace)\n(current behavior)"]
    end

    subgraph Data_TaskAndNav["Data (topics + TF + actions)"]
      direction TB
      mapTopic["topic:/map (OccupancyGrid)"]
      tfTopic["topic:/tf + /tf_static"]

      frontiers["topic:/frontiers (planned)"]
      assignedGoals["topic:/assigned_waypoints (planned)"]

      navGoalB["action:/blinky/navigate_to_pose (nav2_msgs/NavigateToPose)"]
      navGoalP["action:/pinky/navigate_to_pose (nav2_msgs/NavigateToPose)"]
      navResult["action feedback/result (success/failure/blocked)"]
      cmdVel["topic:/<robot>/cmd_vel (geometry_msgs/Twist)\n(emitted where Nav2 runs)"]
    end

    globalMapSrc -->|"topic:/map"| mapTopic
    mapTopic --> frontierDetect
    tfTopic --> frontierDetect

    %% Capstone intended loop
    frontierDetect -->|"topic:/frontiers"| frontiers
    frontiers --> taskAllocator
    taskAllocator -->|"topic:/assigned_waypoints"| assignedGoals
    assignedGoals -->|"action goals"| nav2B
    assignedGoals -->|"action goals"| nav2P

    nav2B -->|"action:/blinky/navigate_to_pose"| navGoalB
    nav2P -->|"action:/pinky/navigate_to_pose"| navGoalP
    navGoalB --> navResult
    navGoalP --> navResult
    navResult -->|"replan on completion/invalid path"| taskAllocator

    nav2B -->|"topic:/blinky/cmd_vel"| cmdVel
    nav2P -->|"topic:/pinky/cmd_vel"| cmdVel

    %% Current implemented (greedy per-robot)
    mapTopic --> explorerB
    mapTopic --> explorerP
    explorerB -->|"action goals"| nav2B
    explorerP -->|"action goals"| nav2P
  end
```

---

## Function: Object detection + reporting (capstone contract)

```mermaid
flowchart TB
  subgraph Function_ObjectDetection["Function: ObjectDetection+Reporting"]
    direction TB

    subgraph Programs_ObjectDetection["Programs"]
      direction TB
      objDetB["node: ObjectDetector (on robot blinky)\n(model inference)"]
      objDetP["node: ObjectDetector (on robot pinky)\n(model inference)"]
      objFuse["node: ObjectReportAggregator (central)\n(optional: de-dup + global transform)"]
    end

    subgraph Data_ObjectDetection["Data (topics + TF)"]
      direction TB
      imageB["topic:/blinky/camera/image_*"]
      imageP["topic:/pinky/camera/image_*"]
      detB["topic:/blinky/object_detections (planned)"]
      detP["topic:/pinky/object_detections (planned)"]
      tfGlobal["topic:/tf (map->...->base/camera)"]
      globalObjects["topic:/object_markers (visualization_msgs/MarkerArray)"]
    end

    imageB --> objDetB
    imageP --> objDetP
    objDetB -->|"topic:/blinky/object_detections"| detB
    objDetP -->|"topic:/pinky/object_detections"| detP

    detB --> objFuse
    detP --> objFuse
    tfGlobal --> objFuse
    objFuse -->|"topic:/object_markers"| globalObjects
  end
```

---

## Function: GUI (global map + markers + status + video)

```mermaid
flowchart TB
  subgraph Function_GUI["Function: GUI (Operator View)"]
    direction TB

    subgraph Programs_GUI["Programs"]
      direction TB
      guiApp["GUI app (RViz2 / rqt / web UI)\n(planned: custom dashboard)"]
      statusAgg["node: RobotStatusAggregator (central)\n(planned)"]
    end

    subgraph Data_GUI["Data (topics)"]
      direction TB
      mapTopic["topic:/map (OccupancyGrid)"]
      tfTopic["topic:/tf + /tf_static"]
      markers["topic:/object_markers (MarkerArray)"]
      robotStatus["topic:/robot_status (planned)\n(name, IP, battery, link, last_seen)"]
      video["topic:/<robot>/camera/image_* (or compressed)"]
    end

    mapTopic --> guiApp
    tfTopic --> guiApp
    markers --> guiApp

    statusAgg -->|"topic:/robot_status"| robotStatus
    robotStatus --> guiApp

    video --> guiApp
  end
```

---

## Function: Security monitoring (deviation + silence watchdog + quarantine)

```mermaid
flowchart TB
  subgraph Function_Security["Function: SecurityMonitoring"]
    direction TB

    subgraph Programs_Security["Programs"]
      direction TB
      securityMon["node: SecurityMonitor (central)\n(planned)"]
      watchdog["node: RobotWatchdog (central)\n(planned: last-message timers)"]
      navCancel["node: Nav2GoalCanceler (optional)\n(planned: cancel goals on compromise)"]
    end

    subgraph Data_Security["Data (topics/actions)"]
      direction TB
      assigned["topic:/assigned_waypoints (planned)"]
      pose["topic:/tf (robot pose in map frame)"]
      heartbeat["topic:/<robot>/heartbeat (planned)\n(or watchdog critical topics)"]
      alerts["topic:/security_alerts (planned)"]
      stop["topic:/<robot>/cmd_vel (zero) OR service:/emergency_stop (planned)"]
      cancel["action:/<robot>/navigate_to_pose/cancel (planned)"]
    end

    assigned --> securityMon
    pose --> securityMon
    heartbeat --> watchdog

    securityMon -->|"publish alerts"| alerts
    watchdog -->|"robot silent => alert"| alerts

    alerts --> navCancel
    navCancel -->|"cancel current goal"| cancel
    alerts -->|"halt robot"| stop
  end
```
