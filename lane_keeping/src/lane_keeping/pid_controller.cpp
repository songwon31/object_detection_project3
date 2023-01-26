#include "lane_keeping_system/pid_controller.h"
#include <cmath>
#include <iostream>
namespace xycar {
PID::PID(float p_gain, float i_gain, float d_gain)
    : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain) {
  p_error_ = 0.0f;
  i_error_ = 0.0f;
  d_error_ = 0.0f;
}

float PID::getAngle(float angle1) {
  angle = angle1;
  /*
  if (std::abs(angle) < 10)
  {
	i_error_ = 0.0f;
  }
  */
}

float PID::getControlOutput(int error) {
  if (abs(angle)>20) {
    p_gain_ = 0.50f;
    i_gain_ = 0.00037f;
    d_gain_ = 0.25f;
  }
  else 
  {
    p_gain_ = 0.25f;
    i_gain_ = 0.000f;
    d_gain_ = 0.00f;
  }
/*
  p_gain_ = 0.45f;
  i_gain_ = 0.0007f;
  d_gain_ = 0.25f;
*/
  float float_type_error = (float)error;
  d_error_ = float_type_error - p_error_;
  p_error_ = float_type_error;
  
  if (error == 0)
  {
    i_error_ = 0;
  }
  else
  {
    i_error_ += float_type_error;
  }
  
<<<<<<< HEAD
  // i_error_ += float_type_error;
=======
  i_error_ += float_type_error;
>>>>>>> 40e4cd2821e2311cef0b408c43ee99fb5e5e16f2
  if (i_error_ > 50000)
  {
	  i_error_ = 50000;
  }
  else if (i_error_ < -50000)
  {
	  i_error_ = -50000;
  }
  // std::cout << i_error_ << '\n';
  return p_gain_ * p_error_ + i_gain_ * i_error_ + d_gain_ * d_error_;
}
}  // namespace xycar
