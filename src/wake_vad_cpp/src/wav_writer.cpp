#include "wake_vad_cpp/wav_writer.hpp"

#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace std;


namespace wake_vad_cpp
{

namespace
{
void write_u16_le(ofstream & out, uint16_t v)
{
  out.put(static_cast<char>(v & 0xFF));
  out.put(static_cast<char>((v >> 8) & 0xFF));
}

void write_u32_le(ofstream & out, uint32_t v)
{
  out.put(static_cast<char>(v & 0xFF));
  out.put(static_cast<char>((v >> 8) & 0xFF));
  out.put(static_cast<char>((v >> 16) & 0xFF));
  out.put(static_cast<char>((v >> 24) & 0xFF));
}
}  // namespace

WavWriter::WavWriter() = default;

bool WavWriter::ensure_output_dir(const string & dir) const
{
  error_code ec;
  filesystem::create_directories(dir, ec);
  return !ec;
}

string WavWriter::make_output_path(const string & dir) const
{
  /// 녹음 시각(초 단위)을 파일명에 넣어 관리하기 쉬운 WAV 경로 생성
  time_t now = time(nullptr);
  tm tm_now{};
  localtime_r(&now, &tm_now);

  ostringstream oss;
  oss << dir << "/wake_vad_" << put_time(&tm_now, "%Y%m%d_%H%M%S") << ".wav";
  return oss.str();
}

bool WavWriter::write_pcm16_mono(
  const string & file_path,
  const vector<int16_t> & audio,
  int sample_rate) const
{
  /// PCM16 mono 버퍼를 RIFF/WAVE 헤더와 함께 직접 직렬화해 저장
  ofstream out(file_path, ios::binary);
  if (!out.is_open()) {
    return false;
  }

  constexpr uint16_t kChannels = 1;
  constexpr uint16_t kBitsPerSample = 16;
  const uint32_t byte_rate = static_cast<uint32_t>(sample_rate) * kChannels * (kBitsPerSample / 8);
  const uint16_t block_align = static_cast<uint16_t>(kChannels * (kBitsPerSample / 8));
  const uint32_t data_size = static_cast<uint32_t>(audio.size() * sizeof(int16_t));
  const uint32_t riff_size = 36 + data_size;

  out.write("RIFF", 4);
  write_u32_le(out, riff_size);
  out.write("WAVE", 4);

  out.write("fmt ", 4);
  write_u32_le(out, 16);
  write_u16_le(out, 1);
  write_u16_le(out, kChannels);
  write_u32_le(out, static_cast<uint32_t>(sample_rate));
  write_u32_le(out, byte_rate);
  write_u16_le(out, block_align);
  write_u16_le(out, kBitsPerSample);

  out.write("data", 4);
  write_u32_le(out, data_size);
  if (!audio.empty()) {
    out.write(
      reinterpret_cast<const char *>(audio.data()),
      static_cast<streamsize>(data_size));
  }

  return out.good();
}

}  // namespace wake_vad_cpp
