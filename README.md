# GTP AMR Fleet (ROS 2)

A prototype fleet of 1–3 autonomous mobile robots (designed to grow to 10+) that carry
picked items from a Goods-to-Person (GTP) pick station to a sorting / consolidation area.

```
 ready queue ──► GTP dock ──► operator loads ──► DISPATCH ──► drop-off ──► worker unloads ──┐
      ▲                                                                                     │
      └─────────────────────────────────── return ───────────────────────────────────────────┘
```

**Status:** one AMR runs the whole cycle in Gazebo under the fleet manager: it drives
with Nav2 and parks on a floor AprilTag at every place, and the fleet side starts with
one launch file. A robot that fails waits for a person and goes back to work with one
service call. Next: three AMRs.

Design details, the robot state list and the deadlock rules are in
[docs/architecture.md](docs/architecture.md).

## Workspace layout

This repository is the `src/` folder of a colcon workspace (for example `~/amr_ws/src`).

| Package | Build type | Runs on | Status | Responsibility |
|---|---|---|---|---|
| `amr_interfaces` | ament_cmake | everywhere | ✅ done | The contract: messages, services and actions shared by all other packages |
| `amr_fleet_manager` | ament_python | fleet server | ✅ done | Task allocation, dock / drop-off / queue reservations, mock AMR for testing |
| `amr_task_executor` | ament_python | each robot | ✅ done | Mission state machine; serves `execute_task` with Nav2's docking server, then turns in place on the floor tag |
| `amr_docking` | ament_python | each robot | ✅ done | Passes the pose of the AprilTag the camera sees to Nav2's docking server |
| `amr_navigation` | ament_cmake (config only) | each robot | ✅ done | Nav2 parameters (AMCL, MPPI inside a rotation shim, docking server), bringup launch, RViz layout |
| `amr_description` | ament_cmake | each robot | ✅ done | URDF / xacro of the AMR: LiDAR, front camera, Gazebo plugins |
| `amr_gazebo` | ament_cmake | simulation | ✅ done | Warehouse world with floor AprilTags, map, robot spawn, ROS ⇄ Gazebo bridge |
| `amr_bringup` | ament_cmake | both | ✅ done | `fleet.launch.py`: one task executor per robot and the fleet manager; next, a multi-robot Gazebo world |
| `amr_hardware_interface` | ament_cmake (C++) | each robot | ⏳ planned | ros2_control diff-drive, battery, e-stop, bin sensors |
| `amr_station_hmi` | ament_python | GTP station PC | ⏳ planned | Touchscreen / button that calls `/fleet/dispatch_amr` |

Dependency rule: every package may depend on `amr_interfaces`, and no robot-side
package depends on `amr_fleet_manager`. Robots and the fleet manager only talk through
the interfaces below, so either side can be replaced later (for example by Open-RMF).

## Interfaces

| Name | Type | Direction |
|---|---|---|
| `/fleet/dispatch_amr` | `amr_interfaces/srv/DispatchAmr` | station HMI → fleet manager |
| `/<robot_id>/robot_state` | `amr_interfaces/msg/RobotState` | robot → fleet manager, ≥ 1 Hz (also the heartbeat) |
| `/<robot_id>/execute_task` | `amr_interfaces/action/ExecuteTask` | fleet manager → robot, one goal per leg |
| `/<robot_id>/reset` | `std_srvs/srv/Trigger` | person on the floor → robot, after a failure is cleared |

## Quick start 1: simulated fleet, no Gazebo needed

Mock robots that only pretend to drive. Tested on ROS 2 Jazzy; the code only uses APIs
available from Humble to Lyrical.

```bash
mkdir -p ~/amr_ws
git clone https://github.com/mm45561/AMR-gtp-fleet.git ~/amr_ws/src
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_fleet_manager fleet_sim.launch.py
```

The fleet manager finds the robots in queue slots `q1`–`q3` and calls one to `gtp_1`:
the nearest of those that have already reported, so usually `amr_1`, but whichever
mock starts first if it is alone (the log line `-> GO_TO_STATION gtp_1` names it).
About 20 s later the log says `finished GO_TO_STATION at gtp_1`. Then, in a second
terminal (use the robot named in the log):

```bash
source ~/amr_ws/install/setup.bash
ros2 topic echo /amr_1/robot_state --field state      # 3 = DOCKED_AT_GTP
ros2 service call /fleet/dispatch_amr amr_interfaces/srv/DispatchAmr "{station_id: gtp_1}"
```

The robot drives to `sort_1`. As soon as it is 1 m clear of the dock, the next robot is
called to replace it. The first one unloads and then returns to a free queue slot.

## Quick start 2: one AMR in Gazebo, full cycle

Tested on Ubuntu 26.04 with ROS 2 Lyrical, Gazebo Jetty, Nav2 and `apriltag_ros`.
Start each step only when the previous one prints the line shown.

| Step | Terminal | Command | Wait for |
|---|---|---|---|
| 1 | 1 | `ros2 launch amr_navigation bringup.launch.py` | the map in RViz, then 40 s more |
| 2 | 2 | `ros2 launch amr_bringup fleet.launch.py` | `finished GO_TO_STATION at gtp_1` |
| 3 | 3 | `ros2 service call /fleet/dispatch_amr amr_interfaces/srv/DispatchAmr "{station_id: gtp_1}"` | `finished DELIVER at sort_1` |

Step 2 starts the task executor of `amr_1` and the fleet manager together; the list of
robots is `ROBOTS` in `amr_bringup/launch/fleet.launch.py`.

The robot first parks in queue slot `q1`, then at `gtp_1`. After the dispatch it
delivers to `sort_1`, unloads for 4 s, returns to the queue and comes back to `gtp_1`
on its own. To stop, press Ctrl + C in terminal 2, then in terminal 1.

Measured in this simulation:

| What | Result |
|---|---|
| Docking | 5 of 5 places in a full cycle, no retries |
| Final position on the tag | about 1 cm, heading under 0.3° |
| AMCL error at `gtp_1` | 2–4 cm |
| One cycle (dispatch → back at `gtp_1`) | about 2.5 min |

## When a robot fails

A robot that cannot finish a leg (blocked path, tag not found, docking cancelled) goes
to `ERROR` and stops. The fleet manager gives it no more work and keeps every place it
holds, so no other robot is sent there. The log says what to do:

```text
amr_1 failed GO_TO_STATION: docking at gtp_1 failed. It keeps ['dock:gtp_1'] and gets
no more work; clear the problem on the floor, then call /amr_1/reset.
```

Clear the problem, then:

```bash
ros2 service call /amr_1/reset std_srvs/srv/Trigger    # answers 'back to IDLE'
```

The fleet manager logs `amr_1 was reset; it gets work again` and puts the robot back in
the queue: a robot standing at a slot is adopted there, any other is sent to the nearest
free slot (a slot it still holds counts as free). Every other place it held is freed once
the robot is `clearance_m` (1 m) away. A reset is refused when the robot is not in
`ERROR` (`nothing to reset`).

To try it in Quick start 2, make the next leg fail by cancelling its docking goal. This
loop cancels the first docking goal it finds, then stops:

```bash
until ros2 service call /dock_robot/_action/cancel_goal action_msgs/srv/CancelGoal "{}" | grep -F "goals_canceling=[a"; do sleep 1; done
```

## Tests

```bash
cd ~/amr_ws
colcon test --packages-select amr_fleet_manager amr_task_executor && colcon test-result --verbose
```

The allocation logic does not import ROS, so its tests also run on their own:
`cd ~/amr_ws/src/amr_fleet_manager && python3 -m pytest test/test_fleet_core.py`.

## Roadmap

1. Three AMRs in one Gazebo world, each with its own namespace, Nav2 and tag reader.
2. `amr_station_hmi`, then `amr_hardware_interface` for a real robot.
