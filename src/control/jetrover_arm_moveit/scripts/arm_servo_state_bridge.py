#!/usr/bin/env python3

import math
from typing import Dict, List

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState


class ArmServoStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("arm_servo_state_bridge")

        self.declare_parameter("service_name", "/ros_robot_controller/bus_servo/get_state")
        self.declare_parameter("publish_topic", "/moveit_joint_states")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"])
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 10])
        self.declare_parameter("center_ticks", [500.0, 500.0, 500.0, 500.0, 500.0, 500.0])
        self.declare_parameter("joint_signs", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("joint_offsets_rad", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("optional_servo_ids", [10])
        self.declare_parameter("servo_range_deg", 240.0)
        self.declare_parameter("servo_ticks_range", 1000.0)

        self.service_name = str(self.get_parameter("service_name").value)
        self.publish_topic = str(self.get_parameter("publish_topic").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.joint_names = [str(v) for v in self.get_parameter("joint_names").value]
        self.servo_ids = [int(v) for v in self.get_parameter("servo_ids").value]
        self.center_ticks = [float(v) for v in self.get_parameter("center_ticks").value]
        self.joint_signs = [float(v) for v in self.get_parameter("joint_signs").value]
        self.joint_offsets_rad = [float(v) for v in self.get_parameter("joint_offsets_rad").value]
        self.optional_servo_ids = {int(v) for v in self.get_parameter("optional_servo_ids").value}

        servo_range_deg = float(self.get_parameter("servo_range_deg").value)
        servo_ticks_range = float(self.get_parameter("servo_ticks_range").value)
        self.rad_per_tick = math.radians(servo_range_deg) / servo_ticks_range

        n = len(self.joint_names)
        if not (
            n == len(self.servo_ids)
            == len(self.center_ticks)
            == len(self.joint_signs)
            == len(self.joint_offsets_rad)
        ):
            raise RuntimeError("joint_names/servo_ids/center_ticks/joint_signs/joint_offsets_rad length mismatch")

        self.cmd_by_servo_id: Dict[int, GetBusServoCmd] = {}
        for servo_id in self.servo_ids:
            cmd = GetBusServoCmd()
            cmd.id = int(servo_id)
            cmd.get_position = 1
            self.cmd_by_servo_id[servo_id] = cmd

        self.cli = self.create_client(GetBusServoState, self.service_name)
        self.pub = self.create_publisher(JointState, self.publish_topic, 20)
        self._inflight = None
        self._last_pub_time = self.get_clock().now()
        self._last_warn_time = self.get_clock().now()
        self._last_ticks_by_servo: Dict[int, float] = {}
        # MoveIt current_state_monitor expects non-zero, monotonic wall timestamps.
        self._sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"service not ready: {self.service_name}")

        period = max(0.01, 1.0 / max(1.0, self.publish_hz))
        self.timer = self.create_timer(period, self._poll_once)

        self.get_logger().info(
            f"publishing {self.publish_topic} from {self.service_name} at {self.publish_hz:.1f}Hz"
        )

    def _poll_once(self) -> None:
        if self._inflight is not None and not self._inflight.done():
            return
        if not self.cli.service_is_ready():
            return

        req = GetBusServoState.Request()
        req.cmd = [self.cmd_by_servo_id[sid] for sid in self.servo_ids]
        self._inflight = self.cli.call_async(req)
        self._inflight.add_done_callback(self._on_state_response)

    def _on_state_response(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"state query failed: {exc}")
            return

        if not resp.success:
            return

        ticks_by_servo: Dict[int, float] = {}
        for i, st in enumerate(resp.state):
            if not st.position:
                continue
            ticks = float(st.position[-1])
            if st.present_id:
                sid = int(st.present_id[-1])
            elif i < len(self.servo_ids):
                # Some firmware versions leave present_id empty but preserve request order.
                sid = int(self.servo_ids[i])
            else:
                continue
            ticks_by_servo[sid] = ticks

        msg = JointState()
        msg.header.stamp = self._sys_clock.now().to_msg()
        msg.name = list(self.joint_names)

        positions: List[float] = []
        for i, sid in enumerate(self.servo_ids):
            if sid not in ticks_by_servo:
                if sid in self._last_ticks_by_servo:
                    ticks = self._last_ticks_by_servo[sid]
                elif sid in self.optional_servo_ids:
                    # If optional servo is absent, keep neutral for continuity.
                    ticks = self.center_ticks[i]
                else:
                    now = self.get_clock().now()
                    if (now - self._last_warn_time).nanoseconds > int(2e9):
                        missing = [s for s in self.servo_ids if s not in ticks_by_servo and s not in self.optional_servo_ids]
                        self.get_logger().warn(f"missing servo states for ids={missing}; check servo_ids mapping")
                        self._last_warn_time = now
                    return
            else:
                ticks = ticks_by_servo[sid]
                self._last_ticks_by_servo[sid] = ticks
            pos = (
                (ticks - self.center_ticks[i]) * self.rad_per_tick * self.joint_signs[i]
                + self.joint_offsets_rad[i]
            )
            positions.append(pos)

        msg.position = positions
        self.pub.publish(msg)
        self._last_pub_time = self.get_clock().now()


def main() -> None:
    rclpy.init()
    node = ArmServoStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
