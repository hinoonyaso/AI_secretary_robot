#!/usr/bin/env python3

from functools import partial

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class HostSystemMonitor(Node):
    def __init__(self):
        super().__init__("host_system_monitor")

        self.declare_parameter("required_topics", ["/scan", "/odom", "/imu/data"])
        self.declare_parameter("required_tf_pairs", ["odom->base_link", "base_link->laser_frame"])
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("battery_topic", "/battery_node/state")
        self.declare_parameter("battery_stale_sec", 5.0)
        self.declare_parameter("battery_warn_percentage", 0.20)
        self.declare_parameter("require_nav2", False)
        self.declare_parameter(
            "nav2_lifecycle_nodes",
            [
                "/map_server",
                "/amcl",
                "/planner_server",
                "/controller_server",
                "/bt_navigator",
                "/behavior_server",
                "/waypoint_follower",
            ],
        )

        self.required_topics = self.get_parameter("required_topics").value
        self.required_tf_pairs = self.get_parameter("required_tf_pairs").value
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.battery_stale_sec = float(self.get_parameter("battery_stale_sec").value)
        self.battery_warn_percentage = float(self.get_parameter("battery_warn_percentage").value)
        self.require_nav2 = bool(self.get_parameter("require_nav2").value)
        self.nav2_lifecycle_nodes = self.get_parameter("nav2_lifecycle_nodes").value

        self.status_pub = self.create_publisher(String, "/host/system_status", 10)
        self.ok_pub = self.create_publisher(Bool, "/host/system_ok", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_battery_msg = None
        self.last_battery_time = None
        battery_topic = self.get_parameter("battery_topic").value
        self.create_subscription(BatteryState, battery_topic, self._on_battery, 10)

        self.nav2_clients = {
            node_name: self.create_client(GetState, f"{node_name}/get_state")
            for node_name in self.nav2_lifecycle_nodes
        }
        self.nav2_states = {}
        self.nav2_futures = {}

        self.create_timer(2.0, self._tick)
        self.get_logger().info("host_system_monitor started")

    def _on_battery(self, msg: BatteryState):
        self.last_battery_msg = msg
        self.last_battery_time = self.get_clock().now()

    def _parse_tf_pair(self, pair: str):
        if "->" not in pair:
            return None
        src, dst = pair.split("->", 1)
        return src.strip(), dst.strip()

    def _request_nav2_states(self):
        for node_name, client in self.nav2_clients.items():
            if not client.wait_for_service(timeout_sec=0.0):
                continue
            pending = self.nav2_futures.get(node_name)
            if pending is not None and not pending.done():
                continue
            future = client.call_async(GetState.Request())
            future.add_done_callback(partial(self._on_nav2_state, node_name=node_name))
            self.nav2_futures[node_name] = future

    def _on_nav2_state(self, future, node_name: str):
        try:
            response = future.result()
            state_id = int(response.current_state.id)
            state_label = str(response.current_state.label)
            self.nav2_states[node_name] = (state_id, state_label)
        except Exception:
            self.nav2_states[node_name] = (-1, "error")

    def _tick(self):
        self._request_nav2_states()

        topic_names = {name for name, _types in self.get_topic_names_and_types()}
        missing = [t for t in self.required_topics if t not in topic_names]

        missing_tf = []
        for pair in self.required_tf_pairs:
            parsed = self._parse_tf_pair(pair)
            if parsed is None:
                missing_tf.append(f"invalid({pair})")
                continue
            source_frame, target_frame = parsed
            try:
                if not self.tf_buffer.can_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                ):
                    missing_tf.append(pair)
            except TransformException:
                missing_tf.append(pair)

        battery_issues = []
        if self.last_battery_msg is None or self.last_battery_time is None:
            battery_issues.append("battery_topic_no_data")
        else:
            age_sec = (self.get_clock().now() - self.last_battery_time).nanoseconds / 1e9
            if age_sec > self.battery_stale_sec:
                battery_issues.append(f"battery_stale({age_sec:.1f}s)")
            if not self.last_battery_msg.present:
                battery_issues.append("battery_not_present")
            pct = float(self.last_battery_msg.percentage)
            if pct >= 0.0 and pct < self.battery_warn_percentage:
                battery_issues.append(f"battery_low({pct * 100.0:.1f}%)")

        nav2_issues = []
        nav2_services_up = 0
        for node_name, client in self.nav2_clients.items():
            if not client.wait_for_service(timeout_sec=0.0):
                continue
            nav2_services_up += 1
            state = self.nav2_states.get(node_name)
            if state is None:
                nav2_issues.append(f"{node_name}=unknown")
                continue
            state_id, state_label = state
            if state_id != 3:
                nav2_issues.append(f"{node_name}={state_label}")

        if self.require_nav2 and nav2_services_up == 0:
            nav2_issues.append("nav2_lifecycle_services_missing")

        ok = not (missing or missing_tf or battery_issues or nav2_issues)

        status_msg = String()
        if ok:
            status_msg.data = "OK: topics/tf/nav2/battery healthy"
        else:
            details = []
            if missing:
                details.append(f"missing_topics=[{', '.join(missing)}]")
            if missing_tf:
                details.append(f"missing_tf=[{', '.join(missing_tf)}]")
            if nav2_issues:
                details.append(f"nav2=[{', '.join(nav2_issues)}]")
            if battery_issues:
                details.append(f"battery=[{', '.join(battery_issues)}]")
            status_msg.data = "WARN: " + "; ".join(details)
        self.status_pub.publish(status_msg)

        ok_msg = Bool()
        ok_msg.data = ok
        self.ok_pub.publish(ok_msg)

        if ok:
            self.get_logger().info("system check OK")
        else:
            self.get_logger().warn(status_msg.data)


def main():
    rclpy.init()
    node = HostSystemMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
