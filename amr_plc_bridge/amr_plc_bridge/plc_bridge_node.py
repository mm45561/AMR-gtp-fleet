"""
Bridge between the fleet manager and the GTP station PLC (OpenPLC, Modbus TCP).

Every period_s it writes the station's state from /fleet/state into the PLC, and reads
the PLC's dispatch request. A request becomes one /fleet/dispatch_amr call; its result
goes back to the PLC, which then clears the request (the handshake). The heartbeat only
counts up while /fleet/state is fresh, so the PLC's watchdog also notices a silent fleet
manager. Register map: plc_map.py.
"""

from amr_interfaces.msg import FleetState
from amr_interfaces.srv import DispatchAmr
from amr_plc_bridge import plc_map
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time


class PlcBridge(Node):
    """Write the station state into the PLC; turn its DISPATCH requests into service calls."""

    def __init__(self) -> None:
        super().__init__('plc_bridge')
        self._host = self.declare_parameter('plc_host', 'localhost').value
        self._port = self.declare_parameter('plc_port', 502).value
        self._station_id = self.declare_parameter('station_id', 'gtp_1').value
        self._robot_ids = list(self.declare_parameter(
            'robot_ids', ['amr_1', 'amr_2', 'amr_3']).value)
        self._stale_after_s = self.declare_parameter('stale_after_s', 2.0).value
        period = self.declare_parameter('period_s', 0.5).value
        if len(self._robot_ids) > plc_map.PLC_ROBOTS:
            raise ValueError(f'the PLC program has registers for {plc_map.PLC_ROBOTS} robots, '
                             f'got robot_ids={self._robot_ids}')

        self._plc = ModbusTcpClient(self._host, port=self._port)
        self._heartbeat = 0
        self._state: FleetState | None = None
        self._state_time: Time | None = None
        self._dispatching = False       # a /fleet/dispatch_amr call is on its way

        self.create_subscription(FleetState, '/fleet/state', self._on_state, 10)
        self._dispatch = self.create_client(DispatchAmr, '/fleet/dispatch_amr')
        self.create_timer(period, self._on_timer)
        self.get_logger().info(f'PLC bridge up: PLC at {self._host}:{self._port}, '
                               f'station {self._station_id}, robots {self._robot_ids}')

    def destroy_node(self) -> None:
        self._plc.close()
        super().destroy_node()

    def _on_state(self, msg: FleetState) -> None:
        self._state = msg
        self._state_time = self.get_clock().now()

    def _on_timer(self) -> None:
        if not self._plc.connected and not self._plc.connect():
            self.get_logger().warning(
                f'cannot reach the PLC at {self._host}:{self._port}; is OpenPLC running?',
                throttle_duration_sec=10.0)
            return
        try:
            self._write_status()
            self._handle_request()
        except (ModbusException, OSError) as exc:
            self.get_logger().warning(f'Modbus error with the PLC: {exc}',
                                      throttle_duration_sec=10.0)
            self._plc.close()           # connect again on the next period

    # ------------------------------------------------------------- PLC side

    def _write_status(self) -> None:
        """Write %MW0..%MW8; a silent fleet manager reads as 'not ready, no heartbeat'."""
        station, robots = None, []
        if self._fleet_alive():
            self._heartbeat += 1
            station = next((s for s in self._state.stations
                            if s.station_id == self._station_id), None)
            robots = self._state.robots
        values = plc_map.status_block(self._heartbeat, station, robots, self._robot_ids)
        self._check(self._plc.write_registers(plc_map.STATUS_BLOCK, values))

    def _handle_request(self) -> None:
        """Turn a DISPATCH request from the PLC into one /fleet/dispatch_amr call."""
        if self._dispatching:
            return
        response = self._check(self._plc.read_holding_registers(plc_map.DISPATCH_REQUEST))
        if response.registers[0] != 1:
            return
        if not self._dispatch.service_is_ready():
            self.get_logger().warning('DISPATCH pressed, but /fleet/dispatch_amr is not up')
            self._finish(plc_map.RESULT_REJECTED)
            return
        self.get_logger().info(f'DISPATCH pressed at {self._station_id}')
        self._dispatching = True
        request = DispatchAmr.Request()
        request.station_id = self._station_id
        self._dispatch.call_async(request).add_done_callback(self._on_dispatched)

    def _on_dispatched(self, future) -> None:
        self._dispatching = False
        response = future.result()
        # Two calls on purpose: an rclpy logger call site must keep one severity.
        if response.accepted:
            self.get_logger().info(f'dispatch accepted: {response.message}')
        else:
            self.get_logger().warning(f'dispatch rejected: {response.message}')
        try:
            self._finish(plc_map.RESULT_ACCEPTED if response.accepted
                         else plc_map.RESULT_REJECTED)
        except (ModbusException, OSError) as exc:
            self.get_logger().warning(f'could not give the PLC the dispatch result: {exc}')

    def _finish(self, result: int) -> None:
        """Give the PLC the result, then clear its request: the end of the handshake."""
        self._check(self._plc.write_register(plc_map.DISPATCH_RESULT, result))
        self._check(self._plc.write_register(plc_map.DISPATCH_REQUEST, 0))

    # -------------------------------------------------------------- helpers

    def _fleet_alive(self) -> bool:
        if self._state is None or self._state_time is None:
            return False
        age = self.get_clock().now() - self._state_time
        return age.nanoseconds * 1e-9 < self._stale_after_s

    @staticmethod
    def _check(response):
        """Return a Modbus response, or raise if the PLC answered with an error."""
        if response.isError():
            raise ModbusException(str(response))
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PlcBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
