# GTP AMR Fleet (ROS 2)

A prototype fleet of 1–3 autonomous mobile robots (designed to grow to 10+) that carry
picked items from a Goods-to-Person (GTP) pick station to a sorting / consolidation area.

```
 ready queue ──► GTP dock ──► operator loads ──► DISPATCH ──► drop-off ──► worker unloads ──┐
      ▲                                                                                     │
      └─────────────────────────────────── return ───────────────────────────────────────────┘
```

**Status:** three AMRs run the GTP cycle together in Gazebo under the fleet manager. Each
one drives with its own Nav2 under `/amr_N` and parks on a floor AprilTag at every place;
when the docked robot leaves with a delivery, the next one is called to the station. A
robot that fails waits for a person and goes back to work with one service call.

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
| `amr_navigation` | ament_cmake (config only) | each robot | ✅ done | Nav2 parameters (AMCL, MPPI inside a rotation shim, docking server), launch file for one robot's Nav2 and tag reader, RViz layout |
| `amr_description` | ament_cmake | each robot | ✅ done | URDF / xacro of the AMR: LiDAR, front camera, Gazebo plugins; `namespace:=` puts its Gazebo topics under `/amr_N` |
| `amr_gazebo` | ament_cmake | simulation | ✅ done | Warehouse world with floor AprilTags, map; launch file that adds one robot with its own ROS ⇄ Gazebo bridge |
| `amr_bringup` | ament_cmake | both | ✅ done | `sim.launch.py`: Gazebo with 1–3 AMRs, each with its Nav2 under `/amr_N`; `fleet.launch.py`: one task executor per robot and the fleet manager |
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
| `/fleet/state` | `amr_interfaces/msg/FleetState` | fleet manager → station HMI, every tick: who holds each GTP dock, whether DISPATCH would work, every robot's state |

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

The fleet manager waits until all three mock robots have reported (`all 3 robots
reported; work starts`, at most `startup_wait_s` = 10 s), finds them in queue slots
`q1`–`q3` and calls the nearest one, `amr_1`, to `gtp_1`. About 20 s later the log says
`finished GO_TO_STATION at gtp_1`. Then, in a second terminal:

```bash
source ~/amr_ws/install/setup.bash
ros2 topic echo /amr_1/robot_state --field state      # 3 = DOCKED_AT_GTP
ros2 service call /fleet/dispatch_amr amr_interfaces/srv/DispatchAmr "{station_id: gtp_1}"
```

The robot drives to `sort_1`. As soon as it is 1 m clear of the dock, the next robot is
called to replace it. The first one unloads and then returns to a free queue slot.

## Quick start 2: AMRs in Gazebo, full cycle

Tested on Ubuntu 26.04 with ROS 2 Lyrical, Gazebo Jetty, Nav2 and `apriltag_ros`.
Start each step only when the previous one prints the line shown.

| Step | Terminal | Command | Wait for |
|---|---|---|---|
| 1 | 1 | `ros2 launch amr_bringup sim.launch.py` | the map in RViz, then 60 s more |
| 2 | 2 | `ros2 launch amr_bringup fleet.launch.py` | `finished GO_TO_STATION at gtp_1` |
| 3 | 3 | `ros2 service call /fleet/dispatch_amr amr_interfaces/srv/DispatchAmr "{station_id: gtp_1}"` | `finished DELIVER at sort_1` |

Step 1 starts Gazebo and puts `amr_1`, `amr_2` and `amr_3` into queue slots `q1`, `q2`
and `q3`; each robot's Nav2 and tag reader start under its namespace, 5 s apart. RViz
shows `amr_1` (`rviz_robot:=amr_2` shows another one). Step 2 starts the three task
executors and the fleet manager.

The fleet manager waits until every robot has reported (`all 3 robots reported; work
starts`), finds them parked and calls the nearest one to `gtp_1`. After the
dispatch that robot delivers to `sort_1`; as soon as it is 1 m clear of the dock, the
next robot is called to the station. The first one unloads for 4 s and returns to a free
queue slot. To stop, press Ctrl + C in terminal 2, then in terminal 1.

`robots:=1` in steps 1 and 2 runs only `amr_1`, which is lighter on the PC (wait 40 s
after the map instead of 60 s).

Measured in this simulation:

| What | Result |
|---|---|
| Docking | 5 of 5 places in a full cycle, no retries |
| Final position on the tag | about 1 cm, heading under 0.3° |
| AMCL error at `gtp_1` | 2–4 cm |
| One cycle with one AMR (dispatch → back at `gtp_1`) | about 2.5 min |
| Queue slot `q1` → docked at `gtp_1` | about 33 s |
| Three AMRs: Gazebo real-time factor | 99.5 % (RTX 3060) |
| Three AMRs: handoff at `gtp_1` | the next robot is called once the leaving one is 1 m clear |

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

To try it in Quick start 2, make the next leg of `amr_1` fail by cancelling its docking
goal. This loop cancels the first docking goal it finds, then stops:

```bash
until ros2 service call /amr_1/dock_robot/_action/cancel_goal action_msgs/srv/CancelGoal "{}" | grep -F "goals_canceling=[a"; do sleep 1; done
```

## Tests

```bash
cd ~/amr_ws
colcon test --packages-select amr_fleet_manager amr_task_executor && colcon test-result --verbose
```

The allocation logic does not import ROS, so its tests also run on their own:
`cd ~/amr_ws/src/amr_fleet_manager && python3 -m pytest test/test_fleet_core.py`.

## Roadmap

1. `amr_station_hmi`: the station's touchscreen or button that calls `/fleet/dispatch_amr`.
2. `amr_hardware_interface` for a real robot.
