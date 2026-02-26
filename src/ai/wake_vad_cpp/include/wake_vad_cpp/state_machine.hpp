#pragma once

#include <string>

namespace wake_vad_cpp
{

enum class WakeVadState
{
  WAITING,
  LISTENING,
  RECORDING
};

class StateMachine
{
public:
  StateMachine();

  WakeVadState state() const;
  std::string state_string() const;
  void set_state(WakeVadState next);
  void reset();

private:
  WakeVadState state_;
};

}  // namespace wake_vad_cpp

