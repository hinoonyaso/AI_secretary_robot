#!/usr/bin/env python3
import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class TestResult:
    name: str
    ok: bool
    detail: str


def run_cmd(cmd: List[str], timeout: float = 4.0) -> Tuple[int, str, str]:
    try:
        p = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except Exception as e:
        return 1, "", str(e)


def find_jetson_model() -> str:
    path = "/proc/device-tree/model"
    if os.path.exists(path):
        try:
            with open(path, "rb") as f:
                return f.read().replace(b"\x00", b"").decode("utf-8", errors="ignore").strip()
        except Exception:
            pass
    return "unknown"


def test_interfaces() -> List[TestResult]:
    serial_ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    video_ports = sorted(glob.glob("/dev/video*"))

    rc, aplay_out, _ = run_cmd(["arecord", "-l"]) if shutil.which("arecord") else (1, "", "arecord not found")
    audio_detected = rc == 0 and ("card" in aplay_out.lower())

    return [
        TestResult("Serial ports", len(serial_ports) > 0, ", ".join(serial_ports) if serial_ports else "No /dev/ttyUSB* or /dev/ttyACM* found"),
        TestResult("Video ports", len(video_ports) > 0, ", ".join(video_ports) if video_ports else "No /dev/video* found"),
        TestResult("Audio capture devices", audio_detected, "Microphone capture devices found" if audio_detected else "No microphone capture device detected (arecord -l)"),
    ]


def get_ros_topics() -> List[str]:
    if not shutil.which("ros2"):
        return []
    rc, out, _ = run_cmd(["ros2", "topic", "list"], timeout=4.0)
    if rc != 0 or not out:
        return []
    return [x.strip() for x in out.splitlines() if x.strip()]


def test_ros_topic(name: str, topic: str, topics: List[str], reason_if_missing: str) -> TestResult:
    if topic in topics:
        return TestResult(name, True, f"{topic} detected")
    return TestResult(name, False, reason_if_missing)


def test_i2c_mpu6050(buses: List[int]) -> List[TestResult]:
    results = []
    if not shutil.which("i2cdetect"):
        return [TestResult("I2C scan (MPU6050 0x68)", False, "i2cdetect not found. Install: sudo apt install i2c-tools")]

    for b in buses:
        rc, out, err = run_cmd(["i2cdetect", "-y", str(b)], timeout=5.0)
        if rc != 0:
            results.append(TestResult(f"I2C bus {b}", False, err or "scan failed"))
            continue
        found = re.search(r"\b68\b", out) is not None
        results.append(TestResult(f"I2C bus {b} MPU6050@0x68", found, "Detected" if found else "0x68 not found"))
    return results


def detect_lidar_port() -> Optional[str]:
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not candidates:
        return None

    rc, out, _ = run_cmd(["bash", "-lc", "ls -l /dev/serial/by-id 2>/dev/null || true"])
    if rc == 0 and out:
        for line in out.splitlines():
            if "rplidar" in line.lower() or "slamtec" in line.lower():
                m = re.search(r"->\s+(../../[^\s]+)", line)
                if m:
                    dev = os.path.realpath(os.path.join("/dev/serial/by-id", m.group(1)))
                    if os.path.exists(dev):
                        return dev

    return candidates[0]


def test_rplidar_ping(serial_port: Optional[str], baud: int = 115200) -> TestResult:
    if not serial_port:
        return TestResult("RPLIDAR A1 serial", False, "No serial port candidate found")

    try:
        import serial  # type: ignore
    except Exception:
        return TestResult("RPLIDAR A1 serial", False, "pyserial not installed. Install: pip3 install pyserial")

    # RPLIDAR health command: A5 52, expects descriptor header 0xA5 0x5A
    try:
        with serial.Serial(serial_port, baudrate=baud, timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.write(bytes([0xA5, 0x52]))
            time.sleep(0.15)
            data = ser.read(16)
            ok = len(data) >= 2 and data[0] == 0xA5 and data[1] == 0x5A
            if ok:
                return TestResult("RPLIDAR A1 health ping", True, f"Descriptor received on {serial_port}")
            return TestResult("RPLIDAR A1 health ping", False, f"No valid descriptor from {serial_port} (rx={data.hex()})")
    except Exception as e:
        return TestResult("RPLIDAR A1 health ping", False, f"Serial open/read failed on {serial_port}: {e}")


def list_usb_lines() -> List[str]:
    if not shutil.which("lsusb"):
        return []
    rc, out, _ = run_cmd(["lsusb"], timeout=3.0)
    if rc != 0 or not out:
        return []
    return [x.strip() for x in out.splitlines() if x.strip()]


def test_orbbec_usb(usb_lines: List[str]) -> TestResult:
    for line in usb_lines:
        low = line.lower()
        if "orbbec" in low or "2bc5:" in low:
            return TestResult("Orbbec USB detection", True, line)
    return TestResult("Orbbec USB detection", False, "No Orbbec device found in lsusb")


def test_rplidar_usb(usb_lines: List[str]) -> TestResult:
    for line in usb_lines:
        low = line.lower()
        if "slamtec" in low or "rplidar" in low:
            return TestResult("RPLIDAR USB detection", True, line)
    return TestResult("RPLIDAR USB detection", False, "No explicit Slamtec/RPLIDAR usb id found")


def test_camera_capture(max_index: int = 6) -> TestResult:
    try:
        import cv2  # type: ignore
    except Exception:
        return TestResult("Camera frame capture", False, "opencv-python not installed (pip3 install opencv-python)")

    tried = []
    for i in range(max_index + 1):
        dev = f"/dev/video{i}"
        if not os.path.exists(dev):
            continue
        tried.append(dev)
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        cap.release()
        if ok and frame is not None and frame.size > 0:
            return TestResult("Camera frame capture", True, f"Captured frame from {dev} ({frame.shape[1]}x{frame.shape[0]})")

    return TestResult("Camera frame capture", False, f"Failed to capture frame from {', '.join(tried) if tried else 'any /dev/video*'}")


def summarize(results: List[TestResult]) -> int:
    print("\n=== Jetson/Controller Hardware Sensor Test ===")
    fails = 0
    for r in results:
        status = "PASS" if r.ok else "FAIL"
        print(f"[{status}] {r.name}: {r.detail}")
        if not r.ok:
            fails += 1
    print("=============================================\n")
    return fails


def main() -> int:
    parser = argparse.ArgumentParser(description="JetRover sensor/board functional test on Jetson Orin Nano")
    parser.add_argument("--i2c-buses", default="0,1,7", help="Comma-separated I2C bus numbers to scan (default: 0,1,7)")
    parser.add_argument("--lidar-port", default="", help="Force lidar serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--skip-camera", action="store_true", help="Skip OpenCV camera frame test")
    parser.add_argument("--skip-i2c", action="store_true", help="Skip direct I2C MPU6050 scan (JetRover often uses STM32->ROS path)")
    parser.add_argument("--lidar-mode", default="auto", choices=["auto", "serial", "ros"], help="Lidar test mode")
    args = parser.parse_args()

    print(f"Detected platform model: {find_jetson_model()}")

    buses = []
    for x in args.i2c_buses.split(","):
        x = x.strip()
        if x:
            try:
                buses.append(int(x))
            except ValueError:
                pass

    results = []
    results.extend(test_interfaces())
    usb_lines = list_usb_lines()
    ros_topics = get_ros_topics()

    results.append(test_orbbec_usb(usb_lines))
    results.append(test_rplidar_usb(usb_lines))
    results.append(
        test_ros_topic(
            "ROS scan topic",
            "/scan",
            ros_topics,
            "No /scan topic detected (source ROS2 and run lidar node first)",
        )
    )
    results.append(
        test_ros_topic(
            "ROS IMU topic",
            "/ros_robot_controller/imu_raw",
            ros_topics,
            "No /ros_robot_controller/imu_raw topic detected (common for JetRover if driver not running)",
        )
    )

    if not args.skip_i2c:
        results.extend(test_i2c_mpu6050(buses if buses else [1]))

    lidar_port = args.lidar_port.strip() or detect_lidar_port()
    results.append(TestResult("RPLIDAR port selected", lidar_port is not None, lidar_port or "No candidate"))
    if args.lidar_mode == "ros":
        results.append(
            test_ros_topic(
                "RPLIDAR via ROS",
                "/scan",
                ros_topics,
                "No /scan topic detected",
            )
        )
    elif args.lidar_mode == "serial":
        results.append(test_rplidar_ping(lidar_port))
    else:
        if "/scan" in ros_topics:
            results.append(TestResult("RPLIDAR functional", True, "Detected /scan topic"))
        else:
            results.append(test_rplidar_ping(lidar_port))

    if not args.skip_camera:
        video_nodes = sorted(glob.glob("/dev/video*"))
        if video_nodes:
            results.append(test_camera_capture())
        else:
            has_orbbec = any(("orbbec" in x.lower() or "2bc5:" in x.lower()) for x in usb_lines)
            if has_orbbec:
                results.append(TestResult("Camera availability", True, "Orbbec USB detected (may run through SDK without /dev/video*)"))
            else:
                results.append(TestResult("Camera availability", False, "No /dev/video* and no Orbbec USB device detected"))

    fails = summarize(results)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
