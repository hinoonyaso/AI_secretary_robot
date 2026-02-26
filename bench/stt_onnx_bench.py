#!/usr/bin/env python3
"""
STT ONNX 지연시간 측정 - ROS2 AudioBuffer 퍼블리셔
stt_node를 별도 터미널에서 먼저 실행해야 합니다.

사용법:
  터미널 A: LD_LIBRARY_PATH=.../stt_cpp/lib/stt_cpp ros2 run stt_cpp stt_node
  터미널 B: python3 bench/stt_onnx_bench.py
"""
import math
import struct
import sys
import threading
import time

SAMPLE_RATE = 16000
DURATION_SEC = 3.0


def make_pcm_float32(duration: float) -> list[float]:
    """400Hz 사인파 float32 샘플 생성"""
    n = int(SAMPLE_RATE * duration)
    return [math.sin(2 * math.pi * 400 * i / SAMPLE_RATE) * 0.25 for i in range(n)]


def main():
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError:
        print("[오류] rclpy를 찾을 수 없습니다. ROS2 환경을 소싱하세요.")
        sys.exit(1)

    try:
        from ros_robot_controller_msgs.msg import AudioBuffer
    except ImportError:
        print("[오류] ros_robot_controller_msgs를 찾을 수 없습니다.")
        sys.exit(1)

    import subprocess, re

    RUNS = 3
    latencies = []

    class BenchNode(Node):
        def __init__(self):
            super().__init__("stt_bench_node")
            self.result_event = threading.Event()
            self.result_text = ""
            self.t_send = 0.0

            self.pub = self.create_publisher(AudioBuffer, "/wake_vad/audio_buffer", 10)
            self.sub = self.create_subscription(
                String, "/wake_vad/transcript", self._on_result, 10
            )
            self.get_logger().info("bench node 초기화 완료")

        def _on_result(self, msg: String):
            elapsed = time.perf_counter() - self.t_send
            self.result_text = msg.data
            latencies.append(elapsed * 1000)
            self.result_event.set()

        def send_audio(self, samples: list[float]):
            msg = AudioBuffer()
            msg.samples = samples
            msg.sample_rate = SAMPLE_RATE
            msg.channels = 1
            msg.session_id = f"bench_{int(time.time())}"
            self.result_event.clear()
            self.t_send = time.perf_counter()
            self.pub.publish(msg)

    rclpy.init()
    node = BenchNode()
    samples = make_pcm_float32(DURATION_SEC)

    executor_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    executor_thread.start()

    print(f"STT ONNX 지연시간 측정 ({RUNS}회)")
    print(f"오디오: {DURATION_SEC}초, {SAMPLE_RATE}Hz, {len(samples)}샘플")
    print(f"토픽: /wake_vad/audio_buffer → /wake_vad/transcript")
    print()

    time.sleep(1.0)  # 노드 준비 대기

    for i in range(RUNS):
        print(f"run {i+1}/{RUNS}: ", end="", flush=True)

        # tegrastats 시작
        tegra = subprocess.Popen(
            ["tegrastats", "--interval", "200"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )

        node.send_audio(samples)
        got = node.result_event.wait(timeout=30.0)

        # tegrastats 중지 + 파싱
        tegra.terminate()
        tegra.wait(timeout=2)
        tegra_out = tegra.stdout.read() if tegra.stdout else ""
        power_vals = re.findall(r"VDD_IN (\d+)mW", tegra_out)
        avg_power = int(sum(int(v) for v in power_vals) / len(power_vals)) if power_vals else 0

        if got:
            ms = latencies[-1]
            print(f"{ms:.0f}ms | VDD_IN={avg_power}mW | 결과: {node.result_text[:60]}")
        else:
            print(f"타임아웃 | VDD_IN={avg_power}mW")
            latencies.append(float("nan"))

        time.sleep(2.0)

    rclpy.shutdown()

    valid = [x for x in latencies if not math.isnan(x)]
    if valid:
        print()
        print(f"=== STT ONNX 결과 ===")
        print(f"평균: {sum(valid)/len(valid):.0f}ms")
        print(f"최소: {min(valid):.0f}ms")
        print(f"최대: {max(valid):.0f}ms")
    else:
        print("측정 실패 - stt_node가 실행 중인지 확인하세요")


if __name__ == "__main__":
    main()
