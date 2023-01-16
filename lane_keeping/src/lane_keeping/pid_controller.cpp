#include "lane_keeping_system/pid_controller.h"
#include <cmath>
namespace xycar
{
PID::PID(float p_gain, float i_gain, float d_gain) : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain)
{
  p_error_ = 0.0f;
  i_error_ = 0.0f;
  d_error_ = 0.0f;
}

float PID::getAngle(float angle)
{
  current_angle = angle;
}

float PID::getControlOutput(int32_t error)
{
  if (abs(current_angle) > 23)
  {
    p_gain_ = 0.52f;
    i_gain_ = 0.000388f;
    d_gain_ = 0.00f;
  }
  else
  {
    p_gain_ = 0.26f;
    i_gain_ = 0.0000009f;
    d_gain_ = 0.00f;
  }

  float float_type_error = (float)error;
  d_error_ = float_type_error - p_error_;
  p_error_ = float_type_error;
  if (error = 0)
  {
    i_error_ = 0;
  }
  else
  {
    i_error_ += float_type_error;
  }
  return p_gain_ * p_error_ + i_gain_ * i_error_ + d_gain_ * d_error_;
}
}  // namespace xycar
