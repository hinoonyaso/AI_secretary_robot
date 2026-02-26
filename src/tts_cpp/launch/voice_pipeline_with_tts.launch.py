"""
클라우드 전용 음성 파이프라인 (voice_pipeline_with_tts.launch.py)

  Wake   : 로컬 VAD + 키워드 감지    (변경 없음, 하드웨어 의존)
  STT    : Groq Whisper API           환경변수: GROQ_API_KEY
  Intent : Groq LLM (intent 분류)    환경변수: GROQ_API_KEY
  LLM    : Cloud auto                 환경변수: OPENAI_API_KEY → GROQ_API_KEY → GEMINI_API_KEY
  TTS    : Microsoft Edge TTS         (무료, 별도 키 불필요)

사용법:
  export GROQ_API_KEY=gsk_...          # STT + Intent + LLM 3순위 공용
  export OPENAI_API_KEY=sk_...         # LLM 1순위 (선택)
  export GEMINI_API_KEY=AIza...        # LLM 3순위 (선택)
  ros2 launch tts_cpp voice_pipeline_with_tts.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    groq_key   = os.environ.get("GROQ_API_KEY",   "")
    openai_key = os.environ.get("OPENAI_API_KEY",  "")
    gemini_key = os.environ.get("GEMINI_API_KEY",  "")

    stt_params    = os.path.join(get_package_share_directory("stt_cpp"),           "config", "params.yaml")
    router_params = os.path.join(get_package_share_directory("intent_router_cpp"), "config", "params.yaml")
    llm_params    = os.path.join(get_package_share_directory("llm_cpp"),           "config", "params.yaml")
    tts_params    = os.path.join(get_package_share_directory("tts_cpp"),           "config", "params.yaml")

    # ── Wake VAD: 로컬 VAD / 키워드, 클라우드 무관 ──────────────────────────
    wake_vad_launch = PathJoinSubstitution(
        [FindPackageShare("wake_vad_cpp"), "launch", "wake_vad.launch.py"])

    # ── STT: Groq Whisper API ─────────────────────────────────────────────────
    stt_node = Node(
        package="stt_cpp",
        executable="stt_node",
        name="stt_node",
        output="screen",
        parameters=[stt_params, {
            "groq_api_key": groq_key,
        }],
    )

    # ── Intent Router: Groq LLM 기반 분류, 로컬 유사도 비활성화 ─────────────
    router_node = Node(
        package="intent_router_cpp",
        executable="intent_router_node",
        name="intent_router_node",
        output="screen",
        parameters=[router_params, {
            "groq_enabled":             True,
            "groq_api_key":             groq_key,
        }],
    )

    # ── LLM: Cloud auto (OPENAI → Groq → Gemini 순서) ────────────────────────
    llm_node = Node(
        package="llm_cpp",
        executable="llm_node",
        name="llm_node",
        output="screen",
        parameters=[llm_params, {
            "llm_provider":   "auto",
            "openai_api_key": openai_key,
            "groq_api_key":   groq_key,
            "gemini_api_key": gemini_key,
        }],
    )

    # ── TTS: Microsoft Edge TTS (클라우드, 별도 API 키 불필요) ──────────────
    tts_node = Node(
        package="tts_cpp",
        executable="tts_node",
        name="tts_node",
        output="screen",
        parameters=[tts_params, {
            "tts_engine": "edge",
        }],
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(wake_vad_launch)),
        stt_node,
        router_node,
        llm_node,
        tts_node,
    ])
