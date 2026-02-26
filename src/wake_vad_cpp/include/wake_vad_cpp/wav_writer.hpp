#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wake_vad_cpp
{

class WavWriter
{
public:
  WavWriter();

  bool ensure_output_dir(const std::string & dir) const;
  std::string make_output_path(const std::string & dir) const;
  bool write_pcm16_mono(
    const std::string & file_path,
    const std::vector<int16_t> & audio,
    int sample_rate) const;
};

}  // namespace wake_vad_cpp

