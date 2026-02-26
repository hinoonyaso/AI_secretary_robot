#include "board.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;


namespace
{

constexpr uint8_t kCrc8Table[256] = {
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

speed_t baud_to_speed(int baudrate)
{
  switch (baudrate) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return B1000000;
  }
}

}  // namespace

Board::Board(const string & device, int baudrate, double timeout_sec)
: device_(device), baudrate_(baudrate), timeout_sec_(timeout_sec)
{
  running_ = true;
  if (!open_port()) {
    throw runtime_error("failed to open serial port: " + device_);
  }
  recv_thread_ = thread(&Board::recv_task, this);
}

Board::~Board()
{
  close();
}

void Board::enable_reception(bool enable)
{
  enable_recv_ = enable;
}

void Board::close()
{
  running_ = false;
  enable_recv_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  lock_guard<mutex> lk(port_mutex_);
  if (fd_ >= 0) {
    tcdrain(fd_);
    tcflush(fd_, TCIOFLUSH);
    ::close(fd_);
    fd_ = -1;
  }
}

bool Board::open_port()
{
  lock_guard<mutex> lk(port_mutex_);

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // 배타적 접근 — 다른 프로세스의 open 차단
  if (ioctl(fd_, TIOCEXCL) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // O_NONBLOCK 해제 → blocking 모드로 전환
  int flags = fcntl(fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // cfmakeraw로 완전한 raw 모드 보장 (잔존 플래그 제거)
  cfmakeraw(&tty);

  cfsetispeed(&tty, baud_to_speed(baudrate_));
  cfsetospeed(&tty, baud_to_speed(baudrate_));

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>(max(1, static_cast<int>(timeout_sec_ * 10.0)));

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

bool Board::reopen_port()
{
  {
    lock_guard<mutex> lk(port_mutex_);
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  for (int i = 0; i < 3 && running_; ++i) {
    if (open_port()) {
      parser_state_ = ParserState::STARTBYTE1;
      frame_.clear();
      recv_count_ = 0;
      return true;
    }
    this_thread::sleep_for(chrono::milliseconds(200));
  }
  return false;
}

void Board::push_frame(uint8_t func, const vector<uint8_t> & data)
{
  lock_guard<mutex> lk(queue_mutex_);
  auto & q = queues_[func];
  if (q.size() >= 10) {
    q.pop_front();
  }
  q.push_back(data);
  queue_cv_.notify_all();
}

optional<vector<uint8_t>> Board::pop_frame(uint8_t func)
{
  lock_guard<mutex> lk(queue_mutex_);
  auto it = queues_.find(func);
  if (it == queues_.end() || it->second.empty()) {
    return nullopt;
  }
  auto data = it->second.front();
  it->second.pop_front();
  return data;
}

optional<vector<uint8_t>> Board::wait_and_pop_frame(uint8_t func, int timeout_ms)
{
  unique_lock<mutex> lk(queue_mutex_);
  const auto ok = queue_cv_.wait_for(
    lk, chrono::milliseconds(timeout_ms),
    [this, func]() {
      auto it = queues_.find(func);
      return it != queues_.end() && !it->second.empty();
    });

  if (!ok) {
    return nullopt;
  }

  auto & q = queues_[func];
  auto data = q.front();
  q.pop_front();
  return data;
}

void Board::recv_task()
{
  static constexpr size_t kBufSize = 256;
  uint8_t buf[kBufSize];

  while (running_) {
    if (!enable_recv_) {
      this_thread::sleep_for(chrono::milliseconds(10));
      continue;
    }

    ssize_t n = 0;
    {
      lock_guard<mutex> lk(port_mutex_);
      if (fd_ < 0) {
        n = -1;
      } else {
        n = ::read(fd_, buf, kBufSize);
      }
    }

    if (n < 0) {
      reopen_port();
      this_thread::sleep_for(chrono::milliseconds(50));
      continue;
    }

    if (n == 0) {
      this_thread::sleep_for(chrono::milliseconds(1));
      continue;
    }

    for (ssize_t i = 0; i < n; ++i) {
      parse_byte(buf[i]);
    }
  }
}

void Board::parse_byte(uint8_t dat)
{
  switch (parser_state_) {
    case ParserState::STARTBYTE1:
      if (dat == 0xAA) {
        parser_state_ = ParserState::STARTBYTE2;
      }
      break;
    case ParserState::STARTBYTE2:
      if (dat == 0x55) {
        parser_state_ = ParserState::FUNCTION;
      } else {
        parser_state_ = ParserState::STARTBYTE1;
      }
      break;
    case ParserState::FUNCTION:
      if (dat < static_cast<uint8_t>(PacketFunction::NONE)) {
        frame_.clear();
        frame_.push_back(dat);
        frame_.push_back(0);
        parser_state_ = ParserState::LENGTH;
      } else {
        parser_state_ = ParserState::STARTBYTE1;
      }
      break;
    case ParserState::LENGTH:
      frame_[1] = dat;
      recv_count_ = 0;
      if (dat == 0) {
        parser_state_ = ParserState::CHECKSUM;
      } else {
        parser_state_ = ParserState::DATA;
      }
      break;
    case ParserState::DATA:
      frame_.push_back(dat);
      recv_count_++;
      if (recv_count_ >= frame_[1]) {
        parser_state_ = ParserState::CHECKSUM;
      }
      break;
    case ParserState::CHECKSUM: {
      const auto crc8 = checksum_crc8(frame_);
      if (crc8 == dat && frame_.size() >= 2) {
        const uint8_t func = frame_[0];
        vector<uint8_t> payload(frame_.begin() + 2, frame_.end());
        push_frame(func, payload);
      }
      parser_state_ = ParserState::STARTBYTE1;
      break;
    }
  }
}

uint8_t Board::checksum_crc8(const vector<uint8_t> & data)
{
  uint8_t check = 0;
  for (auto b : data) {
    check = kCrc8Table[check ^ b];
  }
  return check;
}

void Board::append_u16(vector<uint8_t> & out, uint16_t v)
{
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void Board::append_i16(vector<uint8_t> & out, int16_t v)
{
  append_u16(out, static_cast<uint16_t>(v));
}

void Board::append_f32(vector<uint8_t> & out, float v)
{
  uint8_t bytes[sizeof(float)]{};
  memcpy(bytes, &v, sizeof(float));
  out.insert(out.end(), bytes, bytes + sizeof(float));
}

uint16_t Board::read_u16(const vector<uint8_t> & data, size_t idx)
{
  return static_cast<uint16_t>(data[idx] | (static_cast<uint16_t>(data[idx + 1]) << 8));
}

int16_t Board::read_i16(const vector<uint8_t> & data, size_t idx)
{
  return static_cast<int16_t>(read_u16(data, idx));
}

float Board::read_f32(const vector<uint8_t> & data, size_t idx)
{
  float v = 0.0F;
  memcpy(&v, data.data() + idx, sizeof(float));
  return v;
}

void Board::buf_write(PacketFunction func, const vector<uint8_t> & data)
{
  vector<uint8_t> buf;
  buf.reserve(data.size() + 5);
  buf.push_back(0xAA);
  buf.push_back(0x55);
  buf.push_back(static_cast<uint8_t>(func));
  buf.push_back(static_cast<uint8_t>(data.size()));
  buf.insert(buf.end(), data.begin(), data.end());

  vector<uint8_t> crc_data;
  crc_data.reserve(data.size() + 2);
  crc_data.push_back(static_cast<uint8_t>(func));
  crc_data.push_back(static_cast<uint8_t>(data.size()));
  crc_data.insert(crc_data.end(), data.begin(), data.end());
  buf.push_back(checksum_crc8(crc_data));

  bool write_ok = false;
  {
    lock_guard<mutex> lk(port_mutex_);
    if (fd_ >= 0) {
      const auto n = ::write(fd_, buf.data(), buf.size());
      write_ok = (n == static_cast<ssize_t>(buf.size()));
    }
  }

  if (!write_ok && reopen_port()) {
    lock_guard<mutex> lk(port_mutex_);
    if (fd_ >= 0) {
      (void)::write(fd_, buf.data(), buf.size());
    }
  }
}

optional<uint16_t> Board::get_battery()
{
  if (!enable_recv_) {
    return nullopt;
  }
  auto data = pop_frame(static_cast<uint8_t>(PacketFunction::SYS));
  if (!data || data->size() < 3) {
    return nullopt;
  }
  if ((*data)[0] != 0x04) {
    return nullopt;
  }
  return read_u16(*data, 1);
}

optional<pair<uint8_t, uint8_t>> Board::get_button()
{
  if (!enable_recv_) {
    return nullopt;
  }
  auto data = pop_frame(static_cast<uint8_t>(PacketFunction::KEY));
  if (!data || data->size() < 2) {
    return nullopt;
  }

  const uint8_t key_id = (*data)[0];
  const uint8_t key_event = (*data)[1];
  if (key_event == 0x20) {
    return make_pair(key_id, static_cast<uint8_t>(0));
  }
  if (key_event == 0x01) {
    return make_pair(key_id, static_cast<uint8_t>(1));
  }
  return nullopt;
}

optional<array<float, 6>> Board::get_imu()
{
  if (!enable_recv_) {
    return nullopt;
  }
  auto data = pop_frame(static_cast<uint8_t>(PacketFunction::IMU));
  if (!data || data->size() < 24) {
    return nullopt;
  }

  array<float, 6> out{};
  for (size_t i = 0; i < 6; ++i) {
    out[i] = read_f32(*data, i * 4);
  }
  return out;
}

optional<pair<array<float, 8>, array<int32_t, 16>>> Board::get_gamepad()
{
  if (!enable_recv_) {
    return nullopt;
  }
  auto data = pop_frame(static_cast<uint8_t>(PacketFunction::GAMEPAD));
  if (!data || data->size() < 7) {
    return nullopt;
  }

  const uint16_t buttons_mask = read_u16(*data, 0);
  const uint8_t hat = (*data)[2];
  const int8_t lx = static_cast<int8_t>((*data)[3]);
  const int8_t ly = static_cast<int8_t>((*data)[4]);
  const int8_t rx = static_cast<int8_t>((*data)[5]);
  const int8_t ry = static_cast<int8_t>((*data)[6]);

  array<float, 8> axes{};
  array<int32_t, 16> buttons{};

  if (buttons_mask & 0x0002) {axes[4] = 1.0F;}
  if (buttons_mask & 0x0001) {axes[5] = 1.0F;}
  if (buttons_mask & 0x0100) {buttons[0] = 1;}
  if (buttons_mask & 0x0200) {buttons[1] = 1;}
  if (buttons_mask & 0x0800) {buttons[3] = 1;}
  if (buttons_mask & 0x1000) {buttons[4] = 1;}
  if (buttons_mask & 0x4000) {buttons[6] = 1;}
  if (buttons_mask & 0x8000) {buttons[7] = 1;}
  if (buttons_mask & 0x0004) {buttons[10] = 1;}
  if (buttons_mask & 0x0008) {buttons[11] = 1;}

  axes[0] = (lx >= 0) ? (-static_cast<float>(lx) / 127.0F) : (-static_cast<float>(lx) / 128.0F);
  axes[1] = (ly >= 0) ? (static_cast<float>(ly) / 127.0F) : (static_cast<float>(ly) / 128.0F);
  axes[2] = (rx >= 0) ? (-static_cast<float>(rx) / 127.0F) : (-static_cast<float>(rx) / 128.0F);
  axes[3] = (ry >= 0) ? (static_cast<float>(ry) / 127.0F) : (static_cast<float>(ry) / 128.0F);

  if (hat == 9) {axes[6] = 1.0F;}
  else if (hat == 13) {axes[6] = -1.0F;}

  if (hat == 11) {axes[7] = -1.0F;}
  else if (hat == 15) {axes[7] = 1.0F;}

  return make_pair(axes, buttons);
}

optional<vector<float>> Board::get_sbus()
{
  if (!enable_recv_) {
    return nullopt;
  }
  auto data = pop_frame(static_cast<uint8_t>(PacketFunction::SBUS));
  if (!data || data->size() < 36) {
    return nullopt;
  }

  SBusStatus status;
  for (size_t i = 0; i < 16; ++i) {
    status.channels[i] = read_i16(*data, i * 2);
  }
  status.channel_17 = (*data)[32] != 0;
  status.channel_18 = (*data)[33] != 0;
  status.signal_loss = (*data)[34] != 0;
  status.fail_safe = (*data)[35] != 0;

  vector<float> out;
  out.reserve(16);
  if (status.signal_loss) {
    out.assign(16, 0.5F);
    out[4] = 0.0F;
    out[5] = 0.0F;
    out[6] = 0.0F;
    out[7] = 0.0F;
  } else {
    for (const auto ch : status.channels) {
      out.push_back(static_cast<float>(2.0 * (static_cast<double>(ch) - 192.0) / (1792.0 - 192.0) - 1.0));
    }
  }
  return out;
}

void Board::set_led(float on_time, float off_time, uint16_t repeat, uint8_t led_id)
{
  vector<uint8_t> data;
  data.push_back(led_id);
  append_u16(data, static_cast<uint16_t>(on_time * 1000.0F));
  append_u16(data, static_cast<uint16_t>(off_time * 1000.0F));
  append_u16(data, repeat);
  buf_write(PacketFunction::LED, data);
}

void Board::set_buzzer(uint16_t freq, float on_time, float off_time, uint16_t repeat)
{
  vector<uint8_t> data;
  append_u16(data, freq);
  append_u16(data, static_cast<uint16_t>(on_time * 1000.0F));
  append_u16(data, static_cast<uint16_t>(off_time * 1000.0F));
  append_u16(data, repeat);
  buf_write(PacketFunction::BUZZER, data);
}

void Board::set_motor_speed(const vector<pair<uint16_t, double>> & speeds)
{
  vector<uint8_t> data;
  data.push_back(0x01);
  data.push_back(static_cast<uint8_t>(speeds.size()));
  for (const auto & s : speeds) {
    data.push_back(static_cast<uint8_t>(s.first - 1));
    append_f32(data, static_cast<float>(s.second));
  }
  buf_write(PacketFunction::MOTOR, data);
}

void Board::set_oled_text(uint8_t line, const string & text)
{
  vector<uint8_t> data;
  data.push_back(line);
  data.push_back(static_cast<uint8_t>(text.size()));
  data.insert(data.end(), text.begin(), text.end());
  buf_write(PacketFunction::OLED, data);
}

void Board::pwm_servo_set_position(double duration, const vector<pair<uint16_t, uint16_t>> & positions)
{
  const auto duration_ms = static_cast<uint16_t>(duration * 1000.0);
  vector<uint8_t> data;
  data.push_back(0x01);
  append_u16(data, duration_ms);
  data.push_back(static_cast<uint8_t>(positions.size()));
  for (const auto & p : positions) {
    data.push_back(static_cast<uint8_t>(p.first));
    append_u16(data, p.second);
  }
  buf_write(PacketFunction::PWM_SERVO, data);
}

void Board::pwm_servo_set_offset(uint8_t servo_id, int16_t offset)
{
  vector<uint8_t> data;
  data.push_back(0x07);
  data.push_back(servo_id);
  data.push_back(static_cast<uint8_t>(offset & 0xFF));
  buf_write(PacketFunction::PWM_SERVO, data);
}

optional<int16_t> Board::pwm_servo_read_offset(uint8_t servo_id)
{
  buf_write(PacketFunction::PWM_SERVO, {0x09, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::PWM_SERVO), 200);
  if (!data || data->size() < 3) {
    return nullopt;
  }
  return static_cast<int8_t>((*data)[2]);
}

optional<uint16_t> Board::pwm_servo_read_position(uint8_t servo_id)
{
  buf_write(PacketFunction::PWM_SERVO, {0x05, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::PWM_SERVO), 200);
  if (!data || data->size() < 4) {
    return nullopt;
  }
  return read_u16(*data, 2);
}

void Board::bus_servo_enable_torque(uint8_t servo_id, bool enable)
{
  buf_write(PacketFunction::BUS_SERVO, {static_cast<uint8_t>(enable ? 0x0B : 0x0C), servo_id});
  this_thread::sleep_for(chrono::milliseconds(20));
}

void Board::bus_servo_set_id(uint8_t servo_id_now, uint8_t servo_id_new)
{
  buf_write(PacketFunction::BUS_SERVO, {0x10, servo_id_now, servo_id_new});
  this_thread::sleep_for(chrono::milliseconds(20));
}

void Board::bus_servo_set_offset(uint8_t servo_id, int16_t offset)
{
  buf_write(PacketFunction::BUS_SERVO, {0x20, servo_id, static_cast<uint8_t>(offset & 0xFF)});
  this_thread::sleep_for(chrono::milliseconds(20));
}

void Board::bus_servo_save_offset(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x24, servo_id});
  this_thread::sleep_for(chrono::milliseconds(20));
}

void Board::bus_servo_set_angle_limit(uint8_t servo_id, const array<uint16_t, 2> & limit)
{
  vector<uint8_t> data{0x30, servo_id};
  append_u16(data, limit[0]);
  append_u16(data, limit[1]);
  buf_write(PacketFunction::BUS_SERVO, data);
  this_thread::sleep_for(chrono::milliseconds(20));
}

void Board::bus_servo_set_vin_limit(uint8_t servo_id, const array<uint16_t, 2> & limit)
{
  vector<uint8_t> data{0x34, servo_id};
  append_u16(data, limit[0]);
  append_u16(data, limit[1]);
  buf_write(PacketFunction::BUS_SERVO, data);
  this_thread::sleep_for(chrono::milliseconds(20));
}

void Board::bus_servo_set_temp_limit(uint8_t servo_id, uint8_t limit)
{
  buf_write(PacketFunction::BUS_SERVO, {0x38, servo_id, limit});
  this_thread::sleep_for(chrono::milliseconds(20));
}

void Board::bus_servo_stop(const vector<uint8_t> & servo_id)
{
  vector<uint8_t> data{0x03, static_cast<uint8_t>(servo_id.size())};
  data.insert(data.end(), servo_id.begin(), servo_id.end());
  buf_write(PacketFunction::BUS_SERVO, data);
}

void Board::bus_servo_set_position(double duration, const vector<pair<uint16_t, uint16_t>> & positions)
{
  const auto duration_ms = static_cast<uint16_t>(duration * 1000.0);
  vector<uint8_t> data;
  data.push_back(0x01);
  append_u16(data, duration_ms);
  data.push_back(static_cast<uint8_t>(positions.size()));
  for (const auto & p : positions) {
    data.push_back(static_cast<uint8_t>(p.first));
    append_u16(data, p.second);
  }
  buf_write(PacketFunction::BUS_SERVO, data);
}

optional<uint8_t> Board::bus_servo_read_id(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x12, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 4 || (*data)[2] != 0) {
    return nullopt;
  }
  return (*data)[3];
}

optional<int16_t> Board::bus_servo_read_offset(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x22, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 4 || (*data)[2] != 0) {
    return nullopt;
  }
  return static_cast<int8_t>((*data)[3]);
}

optional<int16_t> Board::bus_servo_read_position(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x05, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 5 || (*data)[2] != 0) {
    return nullopt;
  }
  return read_i16(*data, 3);
}

optional<uint16_t> Board::bus_servo_read_vin(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x07, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 5 || (*data)[2] != 0) {
    return nullopt;
  }
  return read_u16(*data, 3);
}

optional<uint8_t> Board::bus_servo_read_temp(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x09, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 4 || (*data)[2] != 0) {
    return nullopt;
  }
  return (*data)[3];
}

optional<uint8_t> Board::bus_servo_read_temp_limit(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x3A, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 4 || (*data)[2] != 0) {
    return nullopt;
  }
  return (*data)[3];
}

optional<array<uint16_t, 2>> Board::bus_servo_read_angle_limit(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x32, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 7 || (*data)[2] != 0) {
    return nullopt;
  }
  return array<uint16_t, 2>{read_u16(*data, 3), read_u16(*data, 5)};
}

optional<array<uint16_t, 2>> Board::bus_servo_read_vin_limit(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x36, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 7 || (*data)[2] != 0) {
    return nullopt;
  }
  return array<uint16_t, 2>{read_u16(*data, 3), read_u16(*data, 5)};
}

optional<int16_t> Board::bus_servo_read_torque_state(uint8_t servo_id)
{
  buf_write(PacketFunction::BUS_SERVO, {0x0D, servo_id});
  auto data = wait_and_pop_frame(static_cast<uint8_t>(PacketFunction::BUS_SERVO), 200);
  if (!data || data->size() < 4 || (*data)[2] != 0) {
    return nullopt;
  }
  return static_cast<int8_t>((*data)[3]);
}
