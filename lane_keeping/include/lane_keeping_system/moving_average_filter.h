#ifndef MOVING_AVERAGE_FILTER_H_
#define MOVING_AVERAGE_FILTER_H_
#include <deque>
#include <iostream>
#include <vector>

namespace xycar
{
class MovingAverageFilter final
{
public:
  // Construct a new Moving Average Filter object
  MovingAverageFilter(uint8_t sample_size);
  // Add new data to filter
  void addSample(uint16_t new_sample);
  // Get filtered data
  float getWeightedMovingAverage();

private:
  const uint8_t kSampleSize_;
  std::deque<int> samples_;
  std::vector<int> weight_;
};
}  // namespace xycar
#endif  // MOVING_AVERAGE_FILTER_H_
