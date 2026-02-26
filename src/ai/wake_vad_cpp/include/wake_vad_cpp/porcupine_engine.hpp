#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace wake_vad_cpp
{

class PorcupineEngine
{
public:
  PorcupineEngine();
  ~PorcupineEngine();

  PorcupineEngine(const PorcupineEngine &) = delete;
  PorcupineEngine & operator=(const PorcupineEngine &) = delete;

  bool initialize(
    const std::string & access_key,
    const std::string & keyword_path,
    const std::string & model_path,
    float sensitivity);
  bool process(const std::vector<int16_t> & pcm_frame);
  void shutdown();
  bool initialized() const;
  int frame_length() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  bool initialized_;
  int frame_length_;
};

}  // namespace wake_vad_cpp
