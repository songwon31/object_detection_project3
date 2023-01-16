#include "lane_keeping_system/moving_average_filter.h"

namespace xycar
{
MovingAverageFilter::MovingAverageFilter(uint8_t sample_size) : kSampleSize_(sample_size)
{
  weight_.reserve(kSampleSize_);
  for (uint32_t i = 1; i <= kSampleSize_; ++i)
  {
    weight_.push_back(i);
  }
}

void MovingAverageFilter::addSample(uint16_t new_sample)
{
  samples_.push_back(new_sample);
  if (samples_.size() > kSampleSize_)
  {
    samples_.pop_front();
  }
}

float MovingAverageFilter::getWeightedMovingAverage()
{
  uint16_t sum = 0;
  uint16_t weight_sum = 0;
  for (uint32_t i = 0; i < samples_.size(); ++i)
  {
    sum += samples_[i] * weight_[i];
    weight_sum += weight_[i];
  }
  if (weight_sum == 0)
  {
    throw std::runtime_error("Weight sum is zero");
  }
  return (float)sum / weight_sum;
}
}  // namespace xycar
