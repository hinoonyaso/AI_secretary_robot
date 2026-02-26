#include "wake_vad_cpp/state_machine.hpp"

using namespace std;


namespace wake_vad_cpp
{

StateMachine::StateMachine()
: state_(WakeVadState::WAITING)
{
}

WakeVadState StateMachine::state() const
{
  return state_;
}

string StateMachine::state_string() const
{
  switch (state_) {
    case WakeVadState::WAITING:
      return "waiting";
    case WakeVadState::LISTENING:
      return "listening";
    case WakeVadState::RECORDING:
      return "recording";
    default:
      return "unknown";
  }
}

void StateMachine::set_state(WakeVadState next)
{
  state_ = next;
}

void StateMachine::reset()
{
  state_ = WakeVadState::WAITING;
}

}  // namespace wake_vad_cpp

