#!/usr/bin/env python3

import math
import time
from typing import Dict, List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition


class FollowJointTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("follow_joint_trajectory_bridge")

        self.declare_parameter("publish_topic", "/ros_robot_controller/bus_servo/set_position")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"])
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 10])
        self.declare_parameter("center_ticks", [500.0, 500.0, 500.0, 500.0, 500.0, 500.0])
        self.declare_parameter("joint_signs", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("joint_offsets_rad", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("servo_range_deg", 240.0)
        self.declare_parameter("servo_ticks_range", 1000.0)
        self.declare_parameter("min_segment_duration_sec", 0.04)

        self.publish_topic = str(self.get_parameter("publish_topic").value)
        self.joint_names = [str(v) for v in self.get_parameter("joint_names").value]
        self.servo_ids = [int(v) for v in self.get_parameter("servo_ids").value]
        self.center_ticks = [float(v) for v in self.get_parameter("center_ticks").value]
        self.joint_signs = [float(v) for v in self.get_parameter("joint_signs").value]
        self.joint_offsets_rad = [float(v) for v in self.get_parameter("joint_offsets_rad").value]
        self.min_segment_duration_sec = float(self.get_parameter("min_segment_duration_sec").value)

        servo_range_deg = float(self.get_parameter("servo_range_deg").value)
        servo_ticks_range = float(self.get_parameter("servo_ticks_range").value)
        self.rad_per_tick = math.radians(servo_range_deg) / servo_ticks_range
        self.tick_per_rad = 1.0 / self.rad_per_tick

        n = len(self.joint_names)
        if not (
            n == len(self.servo_ids)
            == len(self.center_ticks)
            == len(self.joint_signs)
            == len(self.joint_offsets_rad)
        ):
            raise RuntimeError("joint_names/servo_ids/center_ticks/joint_signs/joint_offsets_rad length mismatch")

        self.pub = self.create_publisher(ServosPosition, self.publish_topic, 10)

        self._servers = [
            ActionServer(
                self,
                FollowJointTrajectory,
                "follow_joint_trajectory",
                execute_callback=self.execute_cb,
                goal_callback=self.goal_cb,
                cancel_callback=self.cancel_cb,
            ),
            ActionServer(
                self,
                FollowJointTrajectory,
                "arm_controller/follow_joint_trajectory",
                execute_callback=self.execute_cb,
                goal_callback=self.goal_cb,
                cancel_callback=self.cancel_cb,
            ),
        ]

        self.get_logger().info(
            f"bridging FollowJointTrajectory to {self.publish_topic} "
            "on [follow_joint_trajectory, arm_controller/follow_joint_trajectory]"
        )

    def goal_cb(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        traj = goal_request.trajectory
        if not traj.points:
            self.get_logger().warn("rejecting empty trajectory goal")
            return GoalResponse.REJECT
        if not traj.joint_names:
            self.get_logger().warn("rejecting trajectory goal without joint_names")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _duration_to_sec(self, d) -> float:
        return float(d.sec) + float(d.nanosec) * 1e-9

    def _ticks_from_rad(self, joint_name: str, pos_rad: float) -> int:
        i = self.joint_names.index(joint_name)
        sign = self.joint_signs[i]
        if abs(sign) < 1e-9:
            raise RuntimeError(f"joint_signs[{joint_name}] is zero")
        ticks = self.center_ticks[i] + ((pos_rad - self.joint_offsets_rad[i]) * self.tick_per_rad / sign)
        ticks = int(round(max(0.0, min(1000.0, ticks))))
        return ticks

    def _publish_point(self, point_positions: Dict[str, float], duration_sec: float) -> None:
        msg = ServosPosition()
        msg.duration = float(max(self.min_segment_duration_sec, duration_sec))
        for i, joint_name in enumerate(self.joint_names):
            if joint_name not in point_positions:
                continue
            sp = ServoPosition()
            sp.id = int(self.servo_ids[i])
            sp.position = int(self._ticks_from_rad(joint_name, float(point_positions[joint_name])))
            msg.position.append(sp)
        self.pub.publish(msg)

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        traj = goal.trajectory

        index_by_name: Dict[str, int] = {name: i for i, name in enumerate(traj.joint_names)}
        missing = [j for j in self.joint_names if j not in index_by_name]
        if missing:
            self.get_logger().warn(
                f"goal is missing joints {missing}; keeping those servos at previous state"
            )

        start = self.get_clock().now()
        prev_tfs = 0.0

        for point in traj.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "canceled"
                return result

            tfs = self._duration_to_sec(point.time_from_start)
            wait_sec = max(0.0, tfs - (self.get_clock().now() - start).nanoseconds * 1e-9)
            if wait_sec > 0.0:
                time.sleep(wait_sec)

            pos_by_joint = {
                j: point.positions[idx] for j, idx in index_by_name.items() if idx < len(point.positions)
            }
            segment_dt = max(self.min_segment_duration_sec, tfs - prev_tfs)
            self._publish_point(pos_by_joint, segment_dt)
            prev_tfs = tfs

            fb = FollowJointTrajectory.Feedback()
            fb.joint_names = list(traj.joint_names)
            fb.desired = point
            fb.actual = point
            fb.error = point
            goal_handle.publish_feedback(fb)

        # Hold briefly for final segment completion on servo side.
        time.sleep(self.min_segment_duration_sec)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ""
        return result


def main() -> None:
    rclpy.init()
    node = FollowJointTrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
