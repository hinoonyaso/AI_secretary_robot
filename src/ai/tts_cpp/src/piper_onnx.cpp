#include "tts_cpp/piper_onnx.hpp"

#include <algorithm>
#include <cstring>

namespace tts_cpp
{

PiperOnnx::PiperOnnx(const PiperOnnxConfig & cfg)
: config_(cfg)
{
  last_error_.clear();
  ready_ = false;

  if (config_.model_path.empty()) {
    last_error_ = "piper_model_path_empty";
    return;
  }
  if (config_.tokens_path.empty()) {
    last_error_ = "piper_tokens_path_empty";
    return;
  }

  SherpaOnnxOfflineTtsConfig tts_config;
  std::memset(&tts_config, 0, sizeof(tts_config));

  tts_config.model.vits.model = config_.model_path.c_str();
  tts_config.model.vits.lexicon = config_.lexicon_path.empty() ? "" : config_.lexicon_path.c_str();
  tts_config.model.vits.tokens = config_.tokens_path.c_str();
  tts_config.model.vits.data_dir = config_.data_dir.empty() ? "" : config_.data_dir.c_str();
  tts_config.model.vits.noise_scale = 0.667F;
  tts_config.model.vits.noise_scale_w = 0.8F;
  tts_config.model.vits.length_scale = 1.0F;
  tts_config.model.vits.dict_dir = "";

  tts_config.model.num_threads = std::max(1, config_.num_threads);
  tts_config.model.debug = config_.debug ? 1 : 0;
  tts_config.model.provider = "cpu";

  tts_config.rule_fsts = "";
  tts_config.max_num_sentences = 1;
  tts_config.rule_fars = "";
  tts_config.silence_scale = 0.2F;

  tts_ = SherpaOnnxCreateOfflineTts(&tts_config);
  if (tts_ == nullptr) {
    last_error_ = "failed_to_create_sherpa_offline_tts";
    return;
  }

  ready_ = true;
}

PiperOnnx::~PiperOnnx()
{
  if (tts_ != nullptr) {
    SherpaOnnxDestroyOfflineTts(tts_);
    tts_ = nullptr;
  }
}

PiperOnnxResult PiperOnnx::synthesize(const std::string & text) const
{
  PiperOnnxResult out;
  last_error_.clear();

  if (!ready_ || tts_ == nullptr) {
    out.error = "piper_onnx_not_ready";
    last_error_ = out.error;
    return out;
  }

  if (text.empty()) {
    out.error = "empty_text";
    last_error_ = out.error;
    return out;
  }

  const int32_t speaker_id = 0;
  const float speed = 1.0F;
  const SherpaOnnxGeneratedAudio * audio =
    SherpaOnnxOfflineTtsGenerate(tts_, text.c_str(), speaker_id, speed);

  if (audio == nullptr || audio->samples == nullptr || audio->n <= 0 || audio->sample_rate <= 0) {
    out.error = "piper_onnx_generation_failed";
    last_error_ = out.error;
    SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
    return out;
  }

  out.pcm_samples.assign(audio->samples, audio->samples + audio->n);
  out.sample_rate = static_cast<uint32_t>(audio->sample_rate);
  out.ok = true;

  SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
  return out;
}

}  // namespace tts_cpp
