#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

class Board
{
public:
  struct SBusStatus
  {
    std::array<int16_t, 16> channels{};
    bool channel_17{false};
    bool channel_18{false};
    bool signal_loss{true};
    bool fail_safe{false};
  };

  Board(const std::string & device, int baudrate, double timeout_sec = 0.1);
  ~Board();

  Board(const Board &) = delete;
  Board & operator=(const Board &) = delete;

  void enable_reception(bool enable = true);
  void close();

  std::optional<uint16_t> get_battery();
  std::optional<std::pair<uint8_t, uint8_t>> get_button();
  std::optional<std::array<float, 6>> get_imu();
  std::optional<std::vector<uint8_t>> get_motor_raw_frame();
  std::optional<std::pair<std::array<float, 8>, std::array<int32_t, 16>>> get_gamepad();
  std::optional<std::vector<float>> get_sbus();

  void set_led(float on_time, float off_time, uint16_t repeat = 1, uint8_t led_id = 1);
  void set_buzzer(uint16_t freq, float on_time, float off_time, uint16_t repeat = 1);
  void set_motor_id_offset(int offset);
  void set_motor_speed(const std::vector<std::pair<uint16_t, double>> & speeds);
  void set_oled_text(uint8_t line, const std::string & text);

  void pwm_servo_set_position(double duration, const std::vector<std::pair<uint16_t, uint16_t>> & positions);
  void pwm_servo_set_offset(uint8_t servo_id, int16_t offset);
  std::optional<int16_t> pwm_servo_read_offset(uint8_t servo_id);
  std::optional<uint16_t> pwm_servo_read_position(uint8_t servo_id);

  void bus_servo_enable_torque(uint8_t servo_id, bool enable);
  void bus_servo_set_id(uint8_t servo_id_now, uint8_t servo_id_new);
  void bus_servo_set_offset(uint8_t servo_id, int16_t offset);
  void bus_servo_save_offset(uint8_t servo_id);
  void bus_servo_set_angle_limit(uint8_t servo_id, const std::array<uint16_t, 2> & limit);
  void bus_servo_set_vin_limit(uint8_t servo_id, const std::array<uint16_t, 2> & limit);
  void bus_servo_set_temp_limit(uint8_t servo_id, uint8_t limit);
  void bus_servo_stop(const std::vector<uint8_t> & servo_id);
  void bus_servo_set_position(double duration, const std::vector<std::pair<uint16_t, uint16_t>> & positions);

  std::optional<uint8_t> bus_servo_read_id(uint8_t servo_id = 254);
  std::optional<int16_t> bus_servo_read_offset(uint8_t servo_id);
  std::optional<int16_t> bus_servo_read_position(uint8_t servo_id);
  std::optional<uint16_t> bus_servo_read_vin(uint8_t servo_id);
  std::optional<uint8_t> bus_servo_read_temp(uint8_t servo_id);
  std::optional<uint8_t> bus_servo_read_temp_limit(uint8_t servo_id);
  std::optional<std::array<uint16_t, 2>> bus_servo_read_angle_limit(uint8_t servo_id);
  std::optional<std::array<uint16_t, 2>> bus_servo_read_vin_limit(uint8_t servo_id);
  std::optional<int16_t> bus_servo_read_torque_state(uint8_t servo_id);

private:
  enum class PacketFunction : uint8_t
  {
    SYS = 0,
    LED = 1,
    BUZZER = 2,
    MOTOR = 3,
    PWM_SERVO = 4,
    BUS_SERVO = 5,
    KEY = 6,
    IMU = 7,
    GAMEPAD = 8,
    SBUS = 9,
    OLED = 10,
    NONE = 11
  };

  enum class ParserState : uint8_t
  {
    STARTBYTE1 = 0,
    STARTBYTE2 = 1,
    FUNCTION = 2,
    LENGTH = 3,
    DATA = 4,
    CHECKSUM = 5
  };

  bool open_port();
  bool reopen_port();
  void recv_task();
  void parse_byte(uint8_t dat);
  void push_frame(uint8_t func, const std::vector<uint8_t> & data);
  std::optional<std::vector<uint8_t>> pop_frame(uint8_t func);
  std::optional<std::vector<uint8_t>> wait_and_pop_frame(uint8_t func, int timeout_ms);

  void buf_write(PacketFunction func, const std::vector<uint8_t> & data);

  static uint8_t checksum_crc8(const std::vector<uint8_t> & data);
  static void append_u16(std::vector<uint8_t> & out, uint16_t v);
  static void append_i16(std::vector<uint8_t> & out, int16_t v);
  static void append_f32(std::vector<uint8_t> & out, float v);
  static uint16_t read_u16(const std::vector<uint8_t> & data, size_t idx);
  static int16_t read_i16(const std::vector<uint8_t> & data, size_t idx);
  static float read_f32(const std::vector<uint8_t> & data, size_t idx);

  std::string device_;
  int baudrate_;
  double timeout_sec_;
  int motor_id_offset_{0};

  int fd_{-1};
  std::mutex port_mutex_;

  std::atomic<bool> running_{false};
  std::atomic<bool> enable_recv_{false};
  std::thread recv_thread_;

  ParserState parser_state_{ParserState::STARTBYTE1};
  std::vector<uint8_t> frame_;
  uint8_t recv_count_{0};

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::unordered_map<uint8_t, std::deque<std::vector<uint8_t>>> queues_;
};
