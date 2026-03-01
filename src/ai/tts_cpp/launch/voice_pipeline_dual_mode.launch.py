"""
듀얼 모드 음성 파이프라인 (voice_pipeline_dual_mode.launch.py)

ai_mode 인수에 따라 오프라인/하이브리드 모드를 선택합니다.

  offline:
    STT    : moonshine-tiny-ko ONNX (로컬 GPU)
    Intent : 키워드 분류 (로컬)
    LLM    : Ollama qwen2.5:1.5b (로컬)
    TTS    : Piper TTS (로컬) → espeak-ng 폴백

  hybrid:
    STT    : auto (moonshine → Groq 폴백)
    Intent : Groq LLM → 키워드 폴백
    LLM    : auto (OpenAI → Groq → Gemini → Ollama 체인)
    TTS    : auto (Edge TTS → Piper → espeak 체인)

  Wake/VAD는 모드에 무관하게 항상 로컬 (Porcupine + Silero VAD)

사용법:
  # 오프라인 모드 (네트워크 없이 동작)
  ros2 launch tts_cpp voice_pipeline_dual_mode.launch.py ai_mode:=offline

  # 하이브리드 모드 (클라우드 우선, 로컬 폴백)
  export GROQ_API_KEY=gsk_...
  export OPENAI_API_KEY=sk_...
  ros2 launch tts_cpp voice_pipeline_dual_mode.launch.py ai_mode:=hybrid
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ── 모드별 엔진 매핑 ──────────────────────────────────────────────────────
MODE_MAP = {
    "offline": {
        "stt_engine": "moonshine_onnx",
        "intent_provider": "keyword",
        "groq_enabled": False,
        "llm_provider": "ollama",
        "tts_engine": "piper",
    },
    "hybrid": {
        "stt_engine": "auto",
        "intent_provider": "auto",
        "groq_enabled": True,
        "llm_provider": "auto",
        "tts_engine": "auto",
    },
}


def _launch_setup(context, *args, **kwargs):
    ai_mode = context.launch_configurations.get("ai_mode", "hybrid")
    cfg = MODE_MAP.get(ai_mode, MODE_MAP["hybrid"])

    groq_key = os.environ.get("GROQ_API_KEY", "")
    openai_key = os.environ.get("OPENAI_API_KEY", "")
    gemini_key = os.environ.get("GEMINI_API_KEY", "")

    stt_params = os.path.join(
        get_package_share_directory("stt_cpp"), "config", "params.yaml"
    )
    router_params = os.path.join(
        get_package_share_directory("intent_router_cpp"), "config", "params.yaml"
    )
    llm_params = os.path.join(
        get_package_share_directory("llm_cpp"), "config", "params.yaml"
    )
    tts_params = os.path.join(
        get_package_share_directory("tts_cpp"), "config", "params.yaml"
    )

    # ── Wake VAD: 항상 로컬 ────────────────────────────────────────────────
    wake_vad_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wake_vad_cpp"), "launch", "wake_vad.launch.py"]
            )
        )
    )

    # ── STT ────────────────────────────────────────────────────────────────
    stt_node = Node(
        package="stt_cpp",
        executable="stt_node",
        name="stt_node",
        output="screen",
        parameters=[
            stt_params,
            {
                "stt_engine": cfg["stt_engine"],
                "groq_api_key": groq_key,
            },
        ],
    )

    # ── Intent Router ──────────────────────────────────────────────────────
    router_node = Node(
        package="intent_router_cpp",
        executable="intent_router_node",
        name="intent_router_node",
        output="screen",
        parameters=[
            router_params,
            {
                "intent_provider": cfg["intent_provider"],
                "groq_enabled": cfg["groq_enabled"],
                "groq_api_key": groq_key,
            },
        ],
    )

    # ── LLM ────────────────────────────────────────────────────────────────
    llm_node = Node(
        package="llm_cpp",
        executable="llm_node",
        name="llm_node",
        output="screen",
        parameters=[
            llm_params,
            {
                "llm_provider": cfg["llm_provider"],
                "openai_api_key": openai_key,
                "groq_api_key": groq_key,
                "gemini_api_key": gemini_key,
            },
        ],
    )

    # ── TTS ────────────────────────────────────────────────────────────────
    tts_node = Node(
        package="tts_cpp",
        executable="tts_node",
        name="tts_node",
        output="screen",
        parameters=[
            tts_params,
            {
                "tts_engine": cfg["tts_engine"],
            },
        ],
    )

    return [wake_vad_launch, stt_node, router_node, llm_node, tts_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ai_mode",
                default_value="offline",
                description="AI pipeline mode: 'offline' or 'hybrid'",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
