#!/usr/bin/env python3
import argparse
import os
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class TestResult:
    name: str
    ok: bool
    detail: str


def ros_import_or_exit():
    try:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from geometry_msgs.msg import Twist  # type: ignore
        from std_msgs.msg import Int32, Float64  # type: ignore
        from sensor_msgs.msg import JointState  # type: ignore
        try:
            from ros_robot_controller_msgs.msg import ServoPosition as RRCServoPosition, ServosPosition as RRCServosPosition  # type: ignore
        except Exception:
            RRCServoPosition = None
            RRCServosPosition = None
        try:
            from servo_controller_msgs.msg import ServoPosition as SCServoPosition, ServosPosition as SCServosPosition  # type: ignore
        except Exception:
            SCServoPosition = None
            SCServosPosition = None
        return rclpy, Node, Twist, Int32, Float64, JointState, RRCServoPosition, RRCServosPosition, SCServoPosition, SCServosPosition
    except Exception as e:
        print(f"[FAIL] ROS2 imports failed: {e}")
        print("Install/Source ROS2 first, then rerun.")
        sys.exit(2)


class ActuatorTester:
    def __init__(
        self,
        NodeCls,
        TwistCls,
        Int32Cls,
        Float64Cls,
        JointStateCls,
        RRCServoPositionCls,
        RRCServosPositionCls,
        SCServoPositionCls,
        SCServosPositionCls,
        node_name: str,
    ):
        self.node = NodeCls(node_name)
        self.Twist = TwistCls
        self.Int32 = Int32Cls
        self.Float64 = Float64Cls
        self.JointState = JointStateCls
        self.RRCServoPosition = RRCServoPositionCls
        self.RRCServosPosition = RRCServosPositionCls
        self.SCServoPosition = SCServoPositionCls
        self.SCServosPosition = SCServosPositionCls

    def has_topic(self, topic_name: str) -> bool:
        names = [n for n, _ in self.node.get_topic_names_and_types()]
        return topic_name in names

    def choose_first_topic(self, topics: List[str]) -> Optional[str]:
        names = [n for n, _ in self.node.get_topic_names_and_types()]
        for topic in topics:
            if topic in names:
                return topic
        return None

    def get_topic_type(self, topic_name: str) -> Optional[str]:
        for n, t in self.node.get_topic_names_and_types():
            if n == topic_name and t:
                return t[0]
        return None

    def publish_cmd_vel(self, topic: str, linear_x: float, angular_z: float, seconds: float = 1.0, hz: float = 20.0):
        pub = self.node.create_publisher(self.Twist, topic, 10)
        msg = self.Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        dt = 1.0 / hz
        count = int(seconds * hz)
        for _ in range(max(1, count)):
            pub.publish(msg)
            time.sleep(dt)

    def stop_cmd_vel(self, topic: str):
        pub = self.node.create_publisher(self.Twist, topic, 10)
        msg = self.Twist()
        pub.publish(msg)
        time.sleep(0.1)

    def publish_float(self, topic: str, value: float, repeat: int = 10):
        pub = self.node.create_publisher(self.Float64, topic, 10)
        msg = self.Float64()
        msg.data = value
        for _ in range(repeat):
            pub.publish(msg)
            time.sleep(0.05)

    def publish_int(self, topic: str, value: int, repeat: int = 10):
        pub = self.node.create_publisher(self.Int32, topic, 10)
        msg = self.Int32()
        msg.data = value
        for _ in range(repeat):
            pub.publish(msg)
            time.sleep(0.05)

    def publish_joint_state(self, topic: str, names: List[str], positions: List[float], repeat: int = 6):
        pub = self.node.create_publisher(self.JointState, topic, 10)
        msg = self.JointState()
        msg.name = names
        msg.position = positions
        for _ in range(repeat):
            pub.publish(msg)
            time.sleep(0.05)

    def publish_servos_position(self, topic: str, topic_type: str, duration: float, id_pos_list: List[tuple], position_unit: str = "pulse"):
        def ros2_pub_once_fallback(payload: str):
            # First try current environment.
            p = subprocess.run(
                ["ros2", "topic", "pub", "--once", "-w", "0", topic, topic_type, payload],
                capture_output=True,
                text=True,
                timeout=4.0,
            )
            if p.returncode == 0:
                return

            # Then try with rover_ws overlay sourced (rover_ws-only workflow).
            candidates = [
                os.path.expanduser("~/rover_ws/install/setup.bash"),
            ]
            source_lines = ["source /opt/ros/humble/setup.bash"]
            for c in candidates:
                if os.path.exists(c):
                    source_lines.append(f"source {shlex.quote(c)}")
            cmd = " && ".join(source_lines) + " && " + " ".join(
                [
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "-w",
                    "0",
                    shlex.quote(topic),
                    shlex.quote(topic_type),
                    shlex.quote(payload),
                ]
            )
            p2 = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=12.0)
            if p2.returncode != 0:
                err = p2.stderr.strip() or p2.stdout.strip() or p.stderr.strip() or p.stdout.strip()
                raise RuntimeError(f"ros2 topic pub failed: {err}")

        if topic_type == "ros_robot_controller_msgs/msg/ServosPosition":
            if self.RRCServoPosition is not None and self.RRCServosPosition is not None:
                pub = self.node.create_publisher(self.RRCServosPosition, topic, 10)
                msg = self.RRCServosPosition()
                msg.duration = float(duration)
                for sid, spos in id_pos_list:
                    s = self.RRCServoPosition()
                    s.id = int(sid)
                    s.position = int(spos)
                    msg.position.append(s)
                for _ in range(5):
                    pub.publish(msg)
                    time.sleep(0.05)
                return
            # Fallback when python message package is unavailable in current environment.
            pos_yaml = ", ".join([f"{{id: {int(sid)}, position: {int(spos)}}}" for sid, spos in id_pos_list])
            payload = f"{{duration: {float(duration)}, position: [{pos_yaml}]}}"
            ros2_pub_once_fallback(payload)
            return
        elif topic_type == "servo_controller_msgs/msg/ServosPosition":
            if self.SCServoPosition is not None and self.SCServosPosition is not None:
                pub = self.node.create_publisher(self.SCServosPosition, topic, 10)
                msg = self.SCServosPosition()
                msg.duration = float(duration)
                msg.position_unit = position_unit
                for sid, spos in id_pos_list:
                    s = self.SCServoPosition()
                    s.id = int(sid)
                    s.position = float(spos)
                    msg.position.append(s)
                for _ in range(5):
                    pub.publish(msg)
                    time.sleep(0.05)
                return
            # Fallback when python message package is unavailable in current environment.
            pos_yaml = ", ".join([f"{{id: {int(sid)}, position: {float(spos)}}}" for sid, spos in id_pos_list])
            payload = f"{{duration: {float(duration)}, position_unit: '{position_unit}', position: [{pos_yaml}]}}"
            ros2_pub_once_fallback(payload)
            return
        else:
            raise RuntimeError(f"Unsupported servo position topic type: {topic_type}")


def summarize(results: List[TestResult]) -> int:
    print("\n=== ROS2 Actuator Functional Test ===")
    fails = 0
    for r in results:
        st = "PASS" if r.ok else "FAIL"
        print(f"[{st}] {r.name}: {r.detail}")
        if not r.ok:
            fails += 1
    print("=====================================\n")
    return fails


def main() -> int:
    parser = argparse.ArgumentParser(description="JetRover ROS2 actuator functional test")
    parser.add_argument("--cmd-vel-topic", default="", help="Twist topic for chassis drive (default: auto-select /controller/cmd_vel, then /cmd_vel)")
    parser.add_argument("--gripper-topic", default="", help="Gripper command topic (default: auto-select /servo_controller, /ros_robot_controller/bus_servo/set_position)")
    parser.add_argument("--arm-joint-topic", default="", help="Arm command topic (default: auto-select /servo_controller, /ros_robot_controller/bus_servo/set_position)")
    parser.add_argument("--require-gripper", action="store_true", help="Fail if gripper topic is missing")
    parser.add_argument("--require-arm", action="store_true", help="Fail if arm joint topic is missing")
    parser.add_argument("--enable-motion", action="store_true", help="Actually command actuators (default: dry-run checks only)")
    parser.add_argument("--linear-x", type=float, default=0.08, help="Linear speed for drive test")
    parser.add_argument("--angular-z", type=float, default=0.0, help="Angular speed for drive test")
    parser.add_argument("--seconds", type=float, default=1.0, help="Duration for drive test")
    parser.add_argument("--gripper-servo-id", type=int, default=10, help="Gripper servo ID for ServosPosition command")
    parser.add_argument("--gripper-open", type=int, default=650, help="Gripper open pulse (0~1000)")
    parser.add_argument("--gripper-close", type=int, default=350, help="Gripper close pulse (0~1000)")
    parser.add_argument("--arm-servo-id", type=int, default=1, help="Arm test servo ID for ServosPosition command")
    parser.add_argument("--arm-delta", type=int, default=40, help="Arm pulse delta around center(500) for motion test")
    parser.add_argument("--joint-controller-topic", default="/joint_controller", help="JointState fallback topic")
    parser.add_argument("--arm-joint-name", default="joint1", help="Joint name for arm fallback on /joint_controller")
    parser.add_argument("--gripper-joint-name", default="r_joint", help="Joint name for gripper fallback on /joint_controller")
    args = parser.parse_args()

    (
        rclpy,
        NodeCls,
        TwistCls,
        Int32Cls,
        Float64Cls,
        JointStateCls,
        RRCServoPositionCls,
        RRCServosPositionCls,
        SCServoPositionCls,
        SCServosPositionCls,
    ) = ros_import_or_exit()
    rclpy.init(args=None)
    tester = ActuatorTester(
        NodeCls,
        TwistCls,
        Int32Cls,
        Float64Cls,
        JointStateCls,
        RRCServoPositionCls,
        RRCServosPositionCls,
        SCServoPositionCls,
        SCServosPositionCls,
        "jetrover_actuator_test",
    )

    results: List[TestResult] = []
    time.sleep(0.5)

    cmd_topic = args.cmd_vel_topic.strip() or tester.choose_first_topic(["/controller/cmd_vel", "/cmd_vel"]) or "/cmd_vel"
    has_cmd = tester.has_topic(cmd_topic)
    gripper_topic = args.gripper_topic.strip() or tester.choose_first_topic(
        ["/servo_controller", "/ros_robot_controller/bus_servo/set_position", "/gripper/command"]
    ) or "/gripper/command"
    arm_topic = args.arm_joint_topic.strip() or tester.choose_first_topic(
        ["/servo_controller", "/ros_robot_controller/bus_servo/set_position", "/arm/joint1/command"]
    ) or "/arm/joint1/command"

    has_gripper = tester.has_topic(gripper_topic)
    has_arm = tester.has_topic(arm_topic)
    gripper_type = tester.get_topic_type(gripper_topic) if has_gripper else None
    arm_type = tester.get_topic_type(arm_topic) if has_arm else None

    results.append(TestResult("Topic check cmd_vel", has_cmd, cmd_topic))
    results.append(
        TestResult(
            "Topic check gripper",
            has_gripper or (not args.require_gripper),
            gripper_topic if has_gripper else f"{gripper_topic} (not running, optional)",
        )
    )
    results.append(
        TestResult(
            "Topic check arm joint",
            has_arm or (not args.require_arm),
            arm_topic if has_arm else f"{arm_topic} (not running, optional)",
        )
    )
    print("Topic checks done. Executing actuator actions...", flush=True)

    if not args.enable_motion:
        results.append(TestResult("Motion execution", True, "Skipped by default. Re-run with --enable-motion"))
        fails = summarize(results)
        tester.node.destroy_node()
        rclpy.shutdown()
        return 1 if fails else 0

    try:
        if has_cmd:
            print(f"Publishing cmd_vel -> {cmd_topic}", flush=True)
            tester.publish_cmd_vel(cmd_topic, args.linear_x, args.angular_z, seconds=args.seconds)
            tester.stop_cmd_vel(cmd_topic)
            results.append(TestResult("Chassis cmd_vel", True, f"Published Twist on {cmd_topic}"))
        else:
            results.append(TestResult("Chassis cmd_vel", False, "cmd_vel topic missing"))

        if has_gripper:
            print(f"Publishing gripper -> {gripper_topic} ({gripper_type})", flush=True)
            if gripper_type in ("ros_robot_controller_msgs/msg/ServosPosition", "servo_controller_msgs/msg/ServosPosition"):
                try:
                    tester.publish_servos_position(gripper_topic, gripper_type, 0.3, [(args.gripper_servo_id, args.gripper_open)])
                    time.sleep(0.2)
                    tester.publish_servos_position(gripper_topic, gripper_type, 0.3, [(args.gripper_servo_id, args.gripper_close)])
                    results.append(TestResult("Gripper command", True, f"Published {gripper_type} on {gripper_topic} (id={args.gripper_servo_id})"))
                except Exception:
                    tester.publish_joint_state(args.joint_controller_topic, [args.gripper_joint_name], [0.6])
                    tester.publish_joint_state(args.joint_controller_topic, [args.gripper_joint_name], [0.1])
                    results.append(TestResult("Gripper command", True, f"Fallback publish JointState on {args.joint_controller_topic} ({args.gripper_joint_name})"))
            elif gripper_type == "std_msgs/msg/Float64":
                tester.publish_float(gripper_topic, 0.2)
                results.append(TestResult("Gripper command", True, f"Published Float64 on {gripper_topic}"))
            elif gripper_type == "std_msgs/msg/Int32":
                tester.publish_int(gripper_topic, 1)
                results.append(TestResult("Gripper command", True, f"Published Int32 on {gripper_topic}"))
            else:
                results.append(TestResult("Gripper command", False, f"Unsupported topic type: {gripper_type}"))
        else:
            if args.require_gripper:
                results.append(TestResult("Gripper command", False, "gripper topic missing"))
            else:
                results.append(TestResult("Gripper command", True, "Skipped (optional topic not running)"))

        if has_arm:
            print(f"Publishing arm -> {arm_topic} ({arm_type})", flush=True)
            if arm_type in ("ros_robot_controller_msgs/msg/ServosPosition", "servo_controller_msgs/msg/ServosPosition"):
                low = max(0, 500 - abs(args.arm_delta))
                high = min(1000, 500 + abs(args.arm_delta))
                try:
                    tester.publish_servos_position(arm_topic, arm_type, 0.4, [(args.arm_servo_id, low)])
                    time.sleep(0.2)
                    tester.publish_servos_position(arm_topic, arm_type, 0.4, [(args.arm_servo_id, high)])
                    results.append(TestResult("Arm joint command", True, f"Published {arm_type} on {arm_topic} (id={args.arm_servo_id})"))
                except Exception:
                    tester.publish_joint_state(args.joint_controller_topic, [args.arm_joint_name], [0.25])
                    tester.publish_joint_state(args.joint_controller_topic, [args.arm_joint_name], [-0.25])
                    results.append(TestResult("Arm joint command", True, f"Fallback publish JointState on {args.joint_controller_topic} ({args.arm_joint_name})"))
            elif arm_type == "std_msgs/msg/Float64":
                tester.publish_float(arm_topic, 0.1)
                results.append(TestResult("Arm joint command", True, f"Published Float64 on {arm_topic}"))
            elif arm_type == "std_msgs/msg/Int32":
                tester.publish_int(arm_topic, 1)
                results.append(TestResult("Arm joint command", True, f"Published Int32 on {arm_topic}"))
            else:
                results.append(TestResult("Arm joint command", False, f"Unsupported topic type: {arm_type}"))
        else:
            if args.require_arm:
                results.append(TestResult("Arm joint command", False, "arm topic missing"))
            else:
                results.append(TestResult("Arm joint command", True, "Skipped (optional topic not running)"))

    except Exception as e:
        results.append(TestResult("Actuator publish", False, str(e)))
    finally:
        if has_cmd:
            try:
                tester.stop_cmd_vel(cmd_topic)
            except Exception:
                pass
        tester.node.destroy_node()
        rclpy.shutdown()

    fails = summarize(results)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
