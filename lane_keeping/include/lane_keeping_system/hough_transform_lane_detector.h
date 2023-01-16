#ifndef HOUGH_TRANSFORM_LANE_DETECTOR_H_
#define HOUGH_TRANSFORM_LANE_DETECTOR_H_
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <deque>
#include <vector>

#include "opencv2/opencv.hpp"

namespace xycar
{
class HoughTransformLaneDetector final
{
public:
  HoughTransformLaneDetector(const YAML::Node& config);
  std::pair<uint16_t, uint16_t> getLanePosition(const cv::Mat& image);
  void getSpeed(uint8_t speed);

private:
  void set(const YAML::Node& config);
  std::pair<std::vector<uint16_t>, std::vector<uint16_t>> divideLines(const std::vector<cv::Vec4i>& lines);
  uint16_t get_line_pos(const std::vector<cv::Vec4i>& lines, const std::vector<uint16_t>& line_index, bool direction);
  std::pair<float, float> get_line_params(const std::vector<cv::Vec4i>& lines, const std::vector<uint16_t>& line_index);
  void addLSample(uint16_t new_sample);
  float getLWeightedMovingAverage();
  void addRSample(uint16_t new_sample);
  float getRWeightedMovingAverage();

private:
  enum kHoughIndex
  {
    x1 = 0,
    y1,
    x2,
    y2
  };
  static const double kHoughRho;
  static const double kHoughTheta;
  uint16_t canny_edge_low_threshold_;
  uint16_t canny_edge_high_threshold_;
  float hough_line_slope_range_;
  uint8_t hough_threshold_;
  uint8_t hough_min_line_length_;
  uint8_t hough_max_line_gap_;

  // roi params
  uint16_t roi_start_height_;
  uint8_t roi_height_;

  // Image parameters
  uint16_t image_width_;
  uint16_t image_height_;
  uint8_t current_speed;

  int32_t left_mean;
  int32_t right_mean;

  const uint16_t lSampleSize_ = 10;
  const uint16_t rSampleSize_ = 10;
  std::vector<uint16_t> l_weight_;
  std::vector<uint16_t> r_weight_;
  std::deque<uint16_t> l_samples_;
  std::deque<uint16_t> r_samples_;

  // line type falg
  static const bool kLeftLane = true;
  static const bool kRightLane = false;

  bool invaild_path_flag;

  bool debug_;
};
}  // namespace xycar
#endif  // HOUGH_TRANSFORM_LANE_DETECTOR_H_
