#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <stdint.h>

namespace xycar
{
class PID final
{
public:
  // Construct a new PID object
  PID(float p_gain, float i_gain, float d_gain);
  // Calculate PID control
  float getControlOutput(int32_t error);

  float getAngle(float angle);

private:
  float current_angle;
  float p_gain_;
  float i_gain_;
  float d_gain_;
  float p_error_;
  float i_error_;
  float d_error_;
};
}  // namespace xycar
#endif  // PID_CONTROLLER_H_
