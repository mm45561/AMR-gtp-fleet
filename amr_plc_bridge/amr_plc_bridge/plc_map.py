"""
Modbus register map of the GTP station PLC, without ROS (see amr_station_plc/station.st).

In OpenPLC the memory word %MWn is holding register 1024 + n. The bridge writes the
status block %MW0..%MW8 every period, writes %MW9 after a dispatch, and reads and
clears %MW11. %MW10 belongs to the HMI and is never touched here.
"""

from dataclasses import dataclass

STATUS_BLOCK = 1024         # %MW0..%MW8, see status_block()
DISPATCH_RESULT = 1033      # %MW9:  bridge -> PLC, RESULT_ACCEPTED or RESULT_REJECTED
HMI_BUTTON = 1034           # %MW10: HMI -> PLC, the DISPATCH button
DISPATCH_REQUEST = 1035     # %MW11: PLC -> bridge, 1 = please dispatch; the bridge writes 0

RESULT_ACCEPTED = 1
RESULT_REJECTED = 2

STATUS_OFFLINE = 0          # robot status in %MW6..%MW8
STATUS_OK = 1
STATUS_FAULTED = 2          # needs a person, then /<robot_id>/reset

PLC_ROBOTS = 3              # the PLC program has registers for amr_1 .. amr_3
HEARTBEAT_WRAP = 1000       # the PLC stores INT (16 bit, signed): stay far below 32767


@dataclass(frozen=True)
class _Robot:
    """The fields of an amr_interfaces/FleetRobot that the PLC needs."""

    state: int
    online: bool
    faulted: bool


def robot_number(robot_id: str) -> int:
    """Return 2 for 'amr_2', and 0 for '' or a name that does not end in _<number>."""
    suffix = robot_id.rpartition('_')[2]
    return int(suffix) if suffix.isdigit() else 0


def status_block(heartbeat: int, station, robots, robot_ids: list[str]) -> list[int]:
    """
    Return the values of %MW0..%MW8 for one write.

    heartbeat, station ready, robot at the dock, the states of robot_ids[0..2], and their
    statuses. `station` (an amr_interfaces/StationState) is None when the fleet manager is
    silent: then the station reads as not ready, so the PLC blocks DISPATCH.
    """
    if len(robot_ids) > PLC_ROBOTS:
        raise ValueError(f'the PLC program has registers for {PLC_ROBOTS} robots, '
                         f'got {len(robot_ids)}: {robot_ids}')
    known = {robot.robot_id: _Robot(robot.state, robot.online, robot.faulted)
             for robot in robots}
    states, statuses = [], []
    for index in range(PLC_ROBOTS):
        robot = known.get(robot_ids[index]) if index < len(robot_ids) else None
        states.append(robot.state if robot else 0)
        statuses.append(_status(robot))
    ready = int(station is not None and station.ready)
    docked = robot_number(station.robot_id) if station is not None else 0
    return [heartbeat % HEARTBEAT_WRAP, ready, docked, *states, *statuses]


def _status(robot: _Robot | None) -> int:
    if robot is None or not robot.online:
        return STATUS_OFFLINE
    return STATUS_FAULTED if robot.faulted else STATUS_OK
