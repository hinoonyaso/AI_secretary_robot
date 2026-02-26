#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace wake_vad_cpp
{

class VadEngine
{
public:
  VadEngine();
  ~VadEngine();

  VadEngine(const VadEngine &) = delete;
  VadEngine & operator=(const VadEngine &) = delete;

  bool initialize(float threshold, const std::string & model_path = "");
  bool is_speech(const std::vector<int16_t> & pcm_frame);
  void reset();
  bool initialized() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  float threshold_;
  bool initialized_;
};

}  // namespace wake_vad_cpp
