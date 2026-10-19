"""
On-robot task executor: runs ExecuteTask goals through the MissionStateMachine.

Every place in the warehouse (GTP station, sort area, queue slots) is a dock of
Nav2's docking server, so one leg of a task is one DockRobot goal: Nav2 drives to
the staging pose, then the docking controller parks on the floor tag.
"""

import math
import threading
import time

from amr_interfaces.action import ExecuteTask
from amr_interfaces.msg import RobotState
from amr_task_executor.mission_state_machine import MissionStateMachine
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import DockRobot
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

# ExecuteTask.task_type -> trigger of the state machine
TASK_TRIGGERS = {
    ExecuteTask.Goal.GO_TO_STATION: 'go_to_station',
    ExecuteTask.Goal.DELIVER: 'deliver',
    ExecuteTask.Goal.RETURN_TO_QUEUE: 'return_to_queue',
}

# dock name -> id of its AprilTag on the floor (see amr_gazebo/worlds/warehouse.sdf)
DOCK_TAGS = {'gtp_1': 0, 'sort_1': 1, 'q1': 2, 'q2': 3, 'q3': 4}


class ExecutorNode(Node):
    """Accept only the tasks the state machine allows, and report the state."""

    def __init__(self) -> None:
        super().__init__('task_executor')
        self._robot_id = (self.declare_parameter('robot_id', '').value
                          or self.get_namespace().strip('/') or 'amr')
        self._unload_time = self.declare_parameter('unload_time_s', 4.0).value
        self._map_frame = self.declare_parameter('map_frame', 'map').value
        self._base_frame = self.declare_parameter('base_frame', 'base_link').value
        # absolute name: in the simulation Nav2 runs without a namespace
        dock_action = self.declare_parameter('dock_action', '/dock_robot').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value

        self._lock = threading.Lock()   # the action thread and the timer share the data
        self._sm = MissionStateMachine()
        self._task_id = ''
        self._busy = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        group = ReentrantCallbackGroup()   # lets the client answer while a task waits
        self._cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self._dock_client = ActionClient(self, DockRobot, dock_action, callback_group=group)

        self._state_pub = self.create_publisher(RobotState, 'robot_state', 10)
        self.create_timer(0.5, self._publish_state)
        self._server = ActionServer(
            self, ExecuteTask, 'execute_task',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=group)
        self.create_service(Trigger, 'reset', self._on_reset)
        self.get_logger().info(f'{self._robot_id} executor ready ({self._sm.state.name})')

    def _on_goal(self, goal: ExecuteTask.Goal) -> GoalResponse:
        trigger = TASK_TRIGGERS.get(goal.task_type)
        with self._lock:
            if trigger is None or self._busy or not self._sm.can(trigger):
                self.get_logger().warning(
                    f'rejected {goal.task_id}: not allowed in {self._sm.state.name}')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _on_reset(self, request: Trigger.Request,
                  response: Trigger.Response) -> Trigger.Response:
        """Leave ERROR once someone has cleared the problem on the floor."""
        with self._lock:
            response.success = not self._busy and self._sm.fire('reset')
            state = self._sm.state.name
        response.message = 'back to IDLE' if response.success else f'nothing to reset ({state})'
        self.get_logger().info(f'reset: {response.message}')
        self._publish_state()
        return response

    def _execute(self, goal_handle) -> ExecuteTask.Result:
        goal = goal_handle.request
        trigger = TASK_TRIGGERS[goal.task_type]
        result = ExecuteTask.Result()
        self.get_logger().info(f'{goal.task_id}: {trigger} -> {goal.target_id}')
        self._fire(trigger, goal.task_id)
        try:
            ok = self._dock_at(goal.target_id)
            if not ok:
                self._fire('fail')
                goal_handle.abort()
                result.message = f'docking at {goal.target_id} failed'
                return result
            self._align_to_tag(goal.target_id)
            self._fire('arrived')
            if trigger == 'deliver':
                time.sleep(self._unload_time)   # stands in for the worker unloading
                self._fire('unloaded')
        finally:
            with self._lock:
                self._busy = False
                self._task_id = ''
        goal_handle.succeed()
        result.success = True
        result.message = f'{trigger} done, now {self._sm.state.name}'
        return result

    def _dock_at(self, dock_id: str) -> bool:
        """Drive to a place and park on its floor tag; return False on failure."""
        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('docking server is not available; is Nav2 active?')
            return False
        goal = DockRobot.Goal()
        goal.dock_id = dock_id      # use_dock_id and navigate_to_staging_pose default to true
        goal_handle = self._wait(self._dock_client.send_goal_async(goal))
        if not goal_handle.accepted:
            self.get_logger().error(f'docking server rejected {dock_id}')
            return False
        result = self._wait(goal_handle.get_result_async()).result
        if not result.success:
            self.get_logger().error(
                f'docking at {dock_id} failed: error {result.error_code} {result.error_msg}')
        return result.success

    def _align_to_tag(self, dock_id: str) -> None:
        """Turn in place until the dock's floor tag is straight ahead (docking leaves ~5 deg)."""
        if dock_id not in DOCK_TAGS:
            return
        tag_frame = f'tag36h11:{DOCK_TAGS[dock_id]}'
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            try:
                tf = self._tf_buffer.lookup_transform(self._base_frame, tag_frame, Time())
            except TransformException:
                break                   # this tag was never seen
            age = self.get_clock().now() - Time.from_msg(tf.header.stamp)
            if age.nanoseconds > 0.5e9:
                break                   # tag out of sight: do not steer on old data
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if abs(yaw) < math.radians(0.3):
                break
            speed = min(0.2, max(0.02, 1.5 * abs(yaw)))   # rad/s, proportional to the error
            self._send_turn(math.copysign(speed, yaw))
            time.sleep(0.05)
        self._send_turn(0.0)
        self.get_logger().info(f'aligned to {tag_frame}')

    def _send_turn(self, angular_z: float) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._base_frame
        cmd.twist.angular.z = angular_z
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _wait(future):
        """Block this task's thread until the executor's other threads finish the future."""
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        done.wait()
        return future.result()

    def _fire(self, trigger: str, task_id: str | None = None) -> None:
        with self._lock:
            before = self._sm.state.name
            self._sm.fire(trigger)
            if task_id is not None:
                self._task_id = task_id
        self.get_logger().info(f'{before} --{trigger}--> {self._sm.state.name}')
        self._publish_state()

    def _publish_state(self) -> None:
        try:    # where the robot really is: the map -> base_link transform (AMCL)
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return      # no pose yet (Nav2 still starting): stay silent, look offline
        msg = RobotState()
        with self._lock:
            msg.state = int(self._sm.state)
            msg.task_id = self._task_id
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.orientation = tf.transform.rotation
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.robot_id = self._robot_id
        msg.battery_percent = 100.0
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecutorNode()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
