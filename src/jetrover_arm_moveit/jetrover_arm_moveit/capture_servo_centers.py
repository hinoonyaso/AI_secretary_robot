#!/usr/bin/env python3

import argparse
from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.node import Node
import yaml

from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState


class ServoCenterCapture(Node):
    def __init__(self, service_name: str) -> None:
        super().__init__("servo_center_capture")
        self._cli = self.create_client(GetBusServoState, service_name)
        self._service_name = service_name

    def wait_ready(self, timeout_sec: float) -> bool:
        return self._cli.wait_for_service(timeout_sec=timeout_sec)

    def read_ticks(self, servo_ids: List[int], timeout_sec: float) -> Dict[int, float]:
        req = GetBusServoState.Request()
        req.cmd = []
        for sid in servo_ids:
            cmd = GetBusServoCmd()
            cmd.id = int(sid)
            cmd.get_position = 1
            req.cmd.append(cmd)

        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError("service call timeout")

        resp = future.result()
        if not resp.success:
            raise RuntimeError("service returned success=false")

        ticks_by_sid: Dict[int, float] = {}
        for st in resp.state:
            if not st.present_id or not st.position:
                continue
            sid = int(st.present_id[-1])
            ticks_by_sid[sid] = float(st.position[-1])

        missing = [sid for sid in servo_ids if sid not in ticks_by_sid]
        if missing:
            raise RuntimeError(f"missing servo states for ids={missing}")

        return {sid: ticks_by_sid[sid] for sid in servo_ids}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture current servo ticks as center_ticks calibration.")
    parser.add_argument("--service", default="/ros_robot_controller/bus_servo/get_state", help="GetBusServoState service name")
    parser.add_argument("--output", default="/home/ubuntu/rover_ws/src/jetrover_arm_moveit/config/servo_calibration.yaml", help="Output YAML path")
    parser.add_argument("--servo-ids", nargs="+", type=int, default=[1, 2, 3, 4, 5, 10], help="Servo IDs in joint order")
    parser.add_argument("--joint-signs", nargs="+", type=float, default=[1, 1, 1, 1, 1, 1], help="Joint signs in joint order")
    parser.add_argument("--joint-offsets-rad", nargs="+", type=float, default=[0, 0, 0, 0, 0, 0], help="Joint offsets in rad")
    parser.add_argument("--wait-sec", type=float, default=5.0, help="Service wait timeout")
    parser.add_argument("--call-timeout-sec", type=float, default=2.0, help="Service call timeout")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    n = len(args.servo_ids)
    if len(args.joint_signs) != n or len(args.joint_offsets_rad) != n:
        print("joint_signs and joint_offsets_rad length must match servo_ids length")
        return 2

    rclpy.init()
    node = ServoCenterCapture(args.service)
    try:
        if not node.wait_ready(args.wait_sec):
            print(f"service not ready: {args.service}")
            return 3

        ticks = node.read_ticks(args.servo_ids, args.call_timeout_sec)
        centers = [float(ticks[sid]) for sid in args.servo_ids]

        out = {
            "arm_servo_state_bridge": {
                "ros__parameters": {
                    "center_ticks": centers,
                    "joint_signs": [float(v) for v in args.joint_signs],
                    "joint_offsets_rad": [float(v) for v in args.joint_offsets_rad],
                }
            }
        }

        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(yaml.safe_dump(out, sort_keys=False), encoding="utf-8")

        print(f"wrote calibration: {out_path}")
        print(f"servo_ids: {args.servo_ids}")
        print(f"center_ticks: {centers}")
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"capture failed: {exc}")
        return 1
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
