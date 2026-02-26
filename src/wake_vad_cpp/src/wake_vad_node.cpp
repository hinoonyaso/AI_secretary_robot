#include "wake_vad_cpp/wake_vad_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>

using namespace std;


namespace wake_vad_cpp
{

/// 노드 초기화: 파라미터 로드 → 퍼블리셔/서브스크라이버 생성 → 엔진 초기화 → 오디오 스트림 시작
WakeVadNode::WakeVadNode()
: Node("wake_vad_node"), audio_device_index_(-1), sensitivity_(0.5), vad_threshold_(0.5),
  silence_duration_(1.5), max_record_duration_(10.0), listen_timeout_(5.0),
  wake_prompt_tts_enabled_(true), wake_prompt_tts_topic_("/llm/response"),
  wake_prompt_tts_text_("네, 말씀하세요."), wake_prompt_wait_tts_done_enabled_(true),
  wake_prompt_wait_timeout_sec_(4.0), tts_playback_done_topic_("/tts/playback_done"),
  save_wav_for_debug_(false), running_(true), waiting_wake_prompt_playback_done_(false),
  silence_tracking_(false)
{
  declare_and_get_parameters();

  pub_detected_ = create_publisher<std_msgs::msg::Bool>("/wake_vad/detected", 10);
  pub_audio_path_ = create_publisher<std_msgs::msg::String>("/wake_vad/audio_path", 10);
  pub_audio_buf_ = create_publisher<ros_robot_controller_msgs::msg::AudioBuffer>(
    "/wake_vad/audio_buffer", 10);
  pub_state_ = create_publisher<std_msgs::msg::String>("/wake_vad/state", 10);
  pub_wake_prompt_tts_ = create_publisher<std_msgs::msg::String>(wake_prompt_tts_topic_, 10);
  sub_tts_playback_done_ = create_subscription<std_msgs::msg::Bool>(
    tts_playback_done_topic_, 10, bind(&WakeVadNode::on_tts_playback_done, this, placeholders::_1));

  wav_writer_.ensure_output_dir(wav_output_dir_);
  initialize_engines();

  processing_thread_ = thread(&WakeVadNode::processing_loop, this);

  audio_input_.configure(audio_device_index_, 16000, 1, 512);
  audio_input_.set_callback(
    bind(&WakeVadNode::on_audio_frame, this, placeholders::_1));
  if (!audio_input_.start()) {
    RCLCPP_ERROR(get_logger(), "failed to start audio input stream");
  }

  publish_state();
  RCLCPP_INFO(get_logger(), "wake_vad_cpp node started");
}

WakeVadNode::~WakeVadNode()
{
  running_.store(false);
  frame_cv_.notify_all();
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  audio_input_.stop();
  porcupine_.shutdown();
}

void WakeVadNode::declare_and_get_parameters()
{
  declare_parameter<string>("access_key", "");
  declare_parameter<string>("keyword_path", "");
  declare_parameter<string>("porcupine_model_path", "");
  declare_parameter<int>("audio_device_index", -1);
  declare_parameter<double>("sensitivity", 0.5);
  declare_parameter<double>("vad_threshold", 0.5);
  declare_parameter<double>("silence_duration", 1.5);
  declare_parameter<double>("max_record_duration", 10.0);
  declare_parameter<string>("wav_output_dir", "/tmp/wake_vad_audio");
  declare_parameter<double>("listen_timeout", 5.0);
  declare_parameter<string>("vad_model_path", "");
  declare_parameter<bool>("wake_prompt_tts_enabled", true);
  declare_parameter<string>("wake_prompt_tts_topic", "/llm/response");
  declare_parameter<string>("wake_prompt_tts_text", "네, 말씀하세요.");
  declare_parameter<bool>("wake_prompt_wait_tts_done_enabled", true);
  declare_parameter<double>("wake_prompt_wait_timeout_sec", 4.0);
  declare_parameter<string>("tts_playback_done_topic", "/tts/playback_done");
  declare_parameter<bool>("save_wav_for_debug", false);

  access_key_ = get_parameter("access_key").as_string();
  keyword_path_ = get_parameter("keyword_path").as_string();
  porcupine_model_path_ = get_parameter("porcupine_model_path").as_string();
  audio_device_index_ = get_parameter("audio_device_index").as_int();
  sensitivity_ = get_parameter("sensitivity").as_double();
  vad_threshold_ = get_parameter("vad_threshold").as_double();
  silence_duration_ = get_parameter("silence_duration").as_double();
  max_record_duration_ = get_parameter("max_record_duration").as_double();
  wav_output_dir_ = get_parameter("wav_output_dir").as_string();
  listen_timeout_ = get_parameter("listen_timeout").as_double();
  vad_model_path_ = get_parameter("vad_model_path").as_string();
  wake_prompt_tts_enabled_ = get_parameter("wake_prompt_tts_enabled").as_bool();
  wake_prompt_tts_topic_ = get_parameter("wake_prompt_tts_topic").as_string();
  wake_prompt_tts_text_ = get_parameter("wake_prompt_tts_text").as_string();
  wake_prompt_wait_tts_done_enabled_ = get_parameter("wake_prompt_wait_tts_done_enabled").as_bool();
  wake_prompt_wait_timeout_sec_ = get_parameter("wake_prompt_wait_timeout_sec").as_double();
  tts_playback_done_topic_ = get_parameter("tts_playback_done_topic").as_string();
  save_wav_for_debug_ = get_parameter("save_wav_for_debug").as_bool();

  // 파라미터에 키가 없으면 환경변수에서 가져옴
  if (access_key_.empty()) {
    const char * env_key = getenv("PICOVOICE_ACCESS_KEY");
    if (env_key) {
      access_key_ = env_key;
    }
  }

  keyword_path_ = resolve_keyword_path(keyword_path_);
  porcupine_model_path_ = resolve_model_path(porcupine_model_path_);
}

/// keyword_path가 파일이면 그대로, 디렉토리면 .ppn 파일을 자동 탐색
string WakeVadNode::resolve_keyword_path(const string & configured_path) const
{
  if (configured_path.empty()) {
    return "";
  }
  error_code ec;
  filesystem::path p(configured_path);
  if (filesystem::is_regular_file(p, ec)) {
    return configured_path;
  }
  if (filesystem::is_directory(p, ec)) {
    vector<string> ppn_files;
    for (const auto & entry : filesystem::directory_iterator(p, ec)) {
      if (entry.is_regular_file() && entry.path().extension() == ".ppn") {
        ppn_files.push_back(entry.path().string());
      }
    }
    sort(ppn_files.begin(), ppn_files.end());
    if (!ppn_files.empty()) {
      if (ppn_files.size() > 1) {
        RCLCPP_WARN(get_logger(), "multiple .ppn files found. using %s", ppn_files.front().c_str());
      }
      return ppn_files.front();
    }
    RCLCPP_WARN(get_logger(), "keyword_path directory has no .ppn file: %s", configured_path.c_str());
    return "";
  }
  RCLCPP_WARN(get_logger(), "keyword_path does not exist: %s", configured_path.c_str());
  return "";
}

string WakeVadNode::resolve_model_path(const string & configured_path) const
{
  if (configured_path.empty()) {
    return "";
  }
  error_code ec;
  if (filesystem::is_regular_file(configured_path, ec)) {
    return configured_path;
  }
  RCLCPP_WARN(get_logger(), "porcupine_model_path does not exist: %s", configured_path.c_str());
  return "";
}

void WakeVadNode::initialize_engines()
{
  const bool porcupine_ok = porcupine_.initialize(
    access_key_, keyword_path_, porcupine_model_path_, static_cast<float>(sensitivity_));
  const bool vad_ok = vad_.initialize(static_cast<float>(vad_threshold_), vad_model_path_);

  RCLCPP_INFO(
    get_logger(), "engine init result - porcupine: %s, vad: %s",
    porcupine_ok ? "ok" : "fail", vad_ok ? "ok" : "fail");
}

/// PortAudio 콜백에서 호출됨 → 프레임을 큐에 넣고 처리 스레드에 통지
/// 큐가 64프레임 초과 시 오래된 프레임 드롭 (실시간 처리 우선)
void WakeVadNode::on_audio_frame(const AudioInput::Frame & frame)
{
  if (!running_.load()) {
    return;
  }

  {
    lock_guard<mutex> lock(frame_mutex_);
    if (frame_queue_.size() > 64) {
      frame_queue_.pop();
    }
    frame_queue_.push(frame);
  }
  frame_cv_.notify_one();
}

/// 프레임 처리 전용 스레드: 큐에서 프레임을 꺼내 현재 상태에 따라 분기
/// WAITING → 웨이크워드 감지 | LISTENING → 음성 시작 대기 | RECORDING → 녹음 + 묵음 감지
void WakeVadNode::processing_loop()
{
  while (running_.load()) {
    AudioInput::Frame frame;
    {
      unique_lock<mutex> lock(frame_mutex_);
      frame_cv_.wait_for(lock, chrono::milliseconds(100), [&]() {
        return !frame_queue_.empty() || !running_.load();
      });
      if (!running_.load()) {
        return;
      }
      if (frame_queue_.empty()) {
        continue;
      }
      frame = move(frame_queue_.front());
      frame_queue_.pop();
    }

    switch (state_machine_.state()) {
      case WakeVadState::WAITING:
        process_waiting(frame);
        break;
      case WakeVadState::LISTENING:
        process_listening(frame);
        break;
      case WakeVadState::RECORDING:
        process_recording(frame);
        break;
      default:
        break;
    }
  }
}

/// WAITING 상태: Porcupine으로 웨이크워드 감지 → 감지 시 "네, 말씀하세요" TTS 발행 후 LISTENING 전환
void WakeVadNode::process_waiting(const AudioInput::Frame & frame)
{
  if (porcupine_.process(frame)) {
    std_msgs::msg::Bool detected_msg;
    detected_msg.data = true;
    pub_detected_->publish(detected_msg);
    RCLCPP_INFO(get_logger(), "wake word detected");

    if (wake_prompt_tts_enabled_ && pub_wake_prompt_tts_ && !wake_prompt_tts_text_.empty()) {
      std_msgs::msg::String tts_prompt_msg;
      tts_prompt_msg.data = wake_prompt_tts_text_;
      pub_wake_prompt_tts_->publish(tts_prompt_msg);
      RCLCPP_INFO(
        get_logger(), "wake prompt published to %s: %s",
        wake_prompt_tts_topic_.c_str(), wake_prompt_tts_text_.c_str());
    }

    if (wake_prompt_tts_enabled_ && wake_prompt_wait_tts_done_enabled_ && !wake_prompt_tts_text_.empty()) {
      waiting_wake_prompt_playback_done_.store(true);
      wake_prompt_wait_start_ = now();
    } else {
      waiting_wake_prompt_playback_done_.store(false);
    }
    listen_start_ = now();
    transition_to(WakeVadState::LISTENING);
  }
}

/// LISTENING 상태: TTS 재생 완료 대기 → VAD로 음성 시작 감지 → RECORDING 전환
/// listen_timeout 초과 시 WAITING으로 복귀
void WakeVadNode::process_listening(const AudioInput::Frame & frame)
{
  // TTS 프롬프트("네, 말씀하세요") 재생이 끝날 때까지 녹음 시작 보류
  if (waiting_wake_prompt_playback_done_.load()) {
    const double waited_sec = (now() - wake_prompt_wait_start_).seconds();
    if (waited_sec < max(0.0, wake_prompt_wait_timeout_sec_)) {
      return;
    }
    waiting_wake_prompt_playback_done_.store(false);
    listen_start_ = now();  // timeout 후 listen window를 새로 시작
    RCLCPP_WARN(
      get_logger(), "wake prompt playback-done wait timeout (%.2fs), continue listening",
      waited_sec);
  }

  if (!vad_.initialized()) {
    record_buffer_.clear();
    record_start_ = now();
    silence_tracking_ = false;
    transition_to(WakeVadState::RECORDING);
    record_buffer_.insert(record_buffer_.end(), frame.begin(), frame.end());
    return;
  }

  const bool speech = vad_.is_speech(frame);
  const double elapsed = (now() - listen_start_).seconds();

  if (speech) {
    RCLCPP_INFO(get_logger(), "speech onset detected");
    record_buffer_.clear();
    record_start_ = now();
    silence_tracking_ = false;
    transition_to(WakeVadState::RECORDING);
    record_buffer_.insert(record_buffer_.end(), frame.begin(), frame.end());
    return;
  }

  if (elapsed >= listen_timeout_) {
    RCLCPP_INFO(get_logger(), "listen timeout. back to waiting");
    vad_.reset();
    transition_to(WakeVadState::WAITING);
  }
}

/// TTS 재생 완료 토픽 수신 시 대기 플래그 해제 → 음성 녹음 시작 허용
void WakeVadNode::on_tts_playback_done(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!waiting_wake_prompt_playback_done_.load()) {
    return;
  }
  waiting_wake_prompt_playback_done_.store(false);
  listen_start_ = now();  // TTS 재생 완료 후 listen window를 새로 시작
  RCLCPP_INFO(
    get_logger(), "wake prompt playback finished (ok=%s), start listening",
    msg->data ? "true" : "false");
}

/// RECORDING 상태: PCM 프레임 누적 + VAD로 묵음 구간 추적
/// silence_duration 이상 묵음 지속 또는 max_record_duration 도달 시 녹음 종료
void WakeVadNode::process_recording(const AudioInput::Frame & frame)
{
  record_buffer_.insert(record_buffer_.end(), frame.begin(), frame.end());

  const double record_elapsed = (now() - record_start_).seconds();
  if (record_elapsed >= max_record_duration_) {
    RCLCPP_INFO(get_logger(), "max record duration reached");
    finalize_recording();
    return;
  }

  const bool speech = vad_.initialized() ? vad_.is_speech(frame) : false;
  if (speech) {
    silence_tracking_ = false;
    return;
  }

  if (!silence_tracking_) {
    silence_tracking_ = true;
    silence_start_ = now();
    return;
  }

  const double silence_elapsed = (now() - silence_start_).seconds();
  if (silence_elapsed >= silence_duration_) {
    RCLCPP_INFO(get_logger(), "silence timeout. saving recording");
    finalize_recording();
  }
}

void WakeVadNode::publish_state()
{
  std_msgs::msg::String state_msg;
  state_msg.data = state_machine_.state_string();
  pub_state_->publish(state_msg);
  RCLCPP_INFO(get_logger(), "state -> %s", state_msg.data.c_str());
}

void WakeVadNode::transition_to(WakeVadState next)
{
  state_machine_.set_state(next);
  publish_state();
}

/// 녹음 완료: AudioBuffer 토픽으로 PCM float32를 직접 발행하고 WAITING 복귀
void WakeVadNode::finalize_recording()
{
  if (record_buffer_.empty()) {
    vad_.reset();
    transition_to(WakeVadState::WAITING);
    return;
  }

  ros_robot_controller_msgs::msg::AudioBuffer buf_msg;
  buf_msg.sample_rate = 16000;
  buf_msg.channels = 1;
  buf_msg.samples.resize(record_buffer_.size());
  for (size_t i = 0; i < record_buffer_.size(); ++i) {
    buf_msg.samples[i] = static_cast<float>(record_buffer_[i]) / 32768.0F;
  }
  pub_audio_buf_->publish(buf_msg);
  RCLCPP_INFO(get_logger(), "audio buffer published: %zu samples", record_buffer_.size());

  if (save_wav_for_debug_) {
    const string file_path = wav_writer_.make_output_path(wav_output_dir_);
    const bool ok = wav_writer_.write_pcm16_mono(file_path, record_buffer_, 16000);
    if (ok) {
      std_msgs::msg::String audio_msg;
      audio_msg.data = file_path;
      pub_audio_path_->publish(audio_msg);
      RCLCPP_INFO(get_logger(), "debug wav saved: %s", file_path.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "failed to write debug audio file: %s", file_path.c_str());
    }
  }

  record_buffer_.clear();
  silence_tracking_ = false;
  vad_.reset();
  transition_to(WakeVadState::WAITING);
}

}  // namespace wake_vad_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<wake_vad_cpp::WakeVadNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
