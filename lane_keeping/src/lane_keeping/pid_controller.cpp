#include "lane_keeping_system/pid_controller.h"
#include <cmath>
namespace xycar {
PID::PID(float p_gain, float i_gain, float d_gain)
    : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain) {
  p_error_ = 0.0f;
  i_error_ = 0.0f;
  d_error_ = 0.0f;
}

float PID::getAngle(float angle1) {
  angle = angle1;
}

float PID::getControlOutput(int error) {
  
  if (abs(angle)>30) {
    p_gain_ = 0.50f;
    i_gain_ = 0.000385f;
    d_gain_ = 0.00f;
  }
  else {
    p_gain_ = 0.30f;
    i_gain_ = 0.000001f;
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
