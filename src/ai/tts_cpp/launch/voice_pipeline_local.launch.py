"""
로컬 전용 음성 파이프라인 런치파일
  - STT  : moonshine-tiny-ko (로컬)
  - LLM  : ollama (로컬, 클라우드 API 스킵)
  - TTS  : MeloTTS → espeak-ng (로컬, edge-tts 스킵)
  - 기타  : wake_vad, intent_router 모두 로컬

사용법:
  ros2 launch tts_cpp voice_pipeline_local.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── wake_vad + STT (기존 파라미터 그대로 사용, stt는 이미 moonshine-tiny-ko) ──
    wake_stt_launch = PathJoinSubstitution(
        [FindPackageShare("stt_cpp"), "launch", "wake_vad_with_stt.launch.py"])

    # ── intent_router (기존 파라미터 그대로 사용, 이미 로컬) ──
    router_launch = PathJoinSubstitution(
        [FindPackageShare("intent_router_cpp"), "launch", "intent_router.launch.py"])

    # ── LLM: ollama 직접 호출 ──
    llm_params = os.path.join(
        get_package_share_directory("llm_cpp"), "config", "params.yaml")

    llm_node = Node(
        package="llm_cpp",
        executable="llm_node",
        name="llm_node",
        output="screen",
        parameters=[llm_params, {"llm_provider": "ollama"}],
    )

    # ── LLM local server (ollama) 상시 대기 ──
    ollama_serve = ExecuteProcess(
        cmd=["ollama", "serve"],
        output="screen",
    )
    ollama_warmup = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "sleep 2; curl -s http://127.0.0.1:11434/api/generate "
            "-H 'Content-Type: application/json' "
            "-d '{\"model\":\"qwen2.5:1.5b\",\"prompt\":\"안녕\",\"stream\":false,\"keep_alive\":\"60m\"}' >/dev/null || true"
        ],
        output="screen",
    )

    # ── TTS: MeloTTS → espeak-ng (edge-tts 스킵) ──
    tts_params = os.path.join(
        get_package_share_directory("tts_cpp"), "config", "params.yaml")

    tts_node = Node(
        package="tts_cpp",
        executable="tts_node",
        name="tts_node",
        output="screen",
        parameters=[tts_params, {"tts_engine": "melo"}],
    )
    tts_share = get_package_share_directory("tts_cpp")
    tts_server = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(tts_share, "scripts", "melo_tts_server.py"),
            "--host",
            "127.0.0.1",
            "--port",
            "5500",
            "--language", "KR",
            "--device", "auto",
        ],
        output="screen",
    )
    delayed_tts_node = TimerAction(period=3.0, actions=[tts_node])

    return LaunchDescription([
        ollama_serve,
        ollama_warmup,
        tts_server,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(wake_stt_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(router_launch)),
        llm_node,
        delayed_tts_node,
    ])
