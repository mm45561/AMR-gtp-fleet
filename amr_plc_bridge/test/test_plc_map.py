"""Unit tests for the PLC register map: no ROS and no PLC needed."""

from types import SimpleNamespace

from amr_plc_bridge import plc_map
import pytest

ROBOTS = ['amr_1', 'amr_2', 'amr_3']


def station(robot_id='', ready=False):
    return SimpleNamespace(station_id='gtp_1', robot_id=robot_id, ready=ready)


def robot(robot_id, state, online=True, faulted=False):
    return SimpleNamespace(robot_id=robot_id, state=state, online=online, faulted=faulted)


def test_addresses_match_station_st():
    # %MW0, %MW9, %MW10 and %MW11 in amr_station_plc/station.st
    assert (plc_map.STATUS_BLOCK, plc_map.DISPATCH_RESULT, plc_map.HMI_BUTTON,
            plc_map.DISPATCH_REQUEST) == (1024, 1033, 1034, 1035)


def test_robot_number():
    assert [plc_map.robot_number(name) for name in ('amr_2', 'amr_12', '', 'agv')] == [
        2, 12, 0, 0]


def test_docked_ready_robot_and_a_faulted_one():
    robots = [robot('amr_1', 3), robot('amr_2', 1), robot('amr_3', 8, faulted=True)]
    values = plc_map.status_block(7, station('amr_1', ready=True), robots, ROBOTS)
    #                 hb ready dock states   statuses
    assert values == [7, 1, 1, 3, 1, 8, 1, 1, 2]


def test_silent_fleet_manager_reads_as_not_ready():
    assert plc_map.status_block(5, None, [], ROBOTS) == [5, 0, 0, 0, 0, 0, 0, 0, 0]


def test_robot_on_its_way_holds_the_dock_but_is_not_ready():
    values = plc_map.status_block(1, station('amr_2'), [robot('amr_2', 2)], ROBOTS)
    assert values[1:3] == [0, 2]


def test_offline_robot_and_fewer_robots_than_plc_registers():
    values = plc_map.status_block(0, station(), [robot('amr_1', 1, online=False)], ['amr_1'])
    assert values == [0, 0, 0, 1, 0, 0, 0, 0, 0]


def test_heartbeat_wraps_below_the_plc_int_range():
    assert plc_map.status_block(1003, None, [], ROBOTS)[0] == 3


def test_too_many_robots_for_the_plc_are_rejected():
    with pytest.raises(ValueError):
        plc_map.status_block(0, None, [], ROBOTS + ['amr_4'])
