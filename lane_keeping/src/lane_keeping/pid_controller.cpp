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

float PID::getDirectionId(int cur_direction_id)
{
  direction_id = cur_direction_id;
  if (direction_id == -1)
  {
    i_error_ = 0.0f;
  }
}

float PID::getControlOutput(int error) {
  if (direction_id >= 0) {
    p_gain_ = 0.30f;
    i_gain_ = 0.0002f;
    d_gain_ = 0.00f;
  } 
  else 
  {
    p_gain_ = 0.23f;
    i_gain_ = 0.000f;
    d_gain_ = 0.00f;
  }
  /*
  if (abs(angle)>20) {
    p_gain_ = 0.45f;
    i_gain_ = 0.0000f;
    d_gain_ = 0.00f;
  } 
  else 
  {
    p_gain_ = 0.20f;
    i_gain_ = 0.000f;
    d_gain_ = 0.00f;
  }
  */
  /*
  p_gain_ = 0.50f;
  i_gain_ = 0.0000f;
  d_gain_ = 0.30f;
  */

  float float_type_error = (float)error;
  d_error_ = float_type_error - p_error_;
  p_error_ = float_type_error;


  i_error_ += float_type_error;

  
  // i_error_ += float_type_error;
  if (i_error_ > 50000)
  {
	  i_error_ = 50000;
   
  }
  else if (i_error_ < -50000)
  {
	  i_error_ = -50000;
  }
  //std::cout << "i_error: " << i_error_ << "| error: " << error << '\n';
  
  return p_gain_ * p_error_ + i_gain_ * i_error_ + d_gain_ * d_error_;
}
}  // namespace xycar
