# One AMR as the fleet manager sees it (part of FleetState).
string robot_id
uint8 state             # a RobotState value: RobotState.IDLE, RobotState.DOCKED_AT_GTP, ...
bool online             # false: its heartbeat stopped
bool faulted            # true: a task failed; clear the problem, then call /<robot_id>/reset
string task             # what it is doing, e.g. 'DELIVER sort_1'; '' = nothing
float32 battery_percent
