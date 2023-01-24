#ifndef HOUGH_TRANSFORM_LANE_DETECTOR_H_
#define HOUGH_TRANSFORM_LANE_DETECTOR_H_
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <deque>
#include <vector>

#include "opencv2/opencv.hpp"

namespace xycar {
class HoughTransformLaneDetector final {
public:
  // Construct a new Hough Transform Lane Detector object
  HoughTransformLaneDetector(const YAML::Node &config);
  // Detect Lane position
  std::pair<int, int> getLanePosition(const cv::Mat &image);
  // Draw rectagles on debug image
  void draw_rectangles(int lpos, int rpos, int ma_mpos);

  void clearSample();
  
  // Get debug image
  cv::Mat *getDebugFrame();

  void getSpeed(int speed1);


  //roi params
  int roi_start_height_;
  int roi_height_;

private:
  // Set parameters from config file
  void set(const YAML::Node &config);
  // Divide lines into left and right
  std::pair<std::vector<int>, std::vector<int>> divideLines(
    const std::vector<cv::Vec4i> &lines);
  // get position of left and right line
  int get_line_pos(const std::vector<cv::Vec4i> &lines,
                   const std::vector<int> &line_index,
                   bool direction);
  // get slpoe and intercept of line
  std::pair<float, float> get_line_params(const std::vector<cv::Vec4i> &lines,
                                          const std::vector<int> &line_index);
  // draw line on debug image
  void draw_lines(const std::vector<cv::Vec4i> &lines,
                  const std::vector<int> &left_line_index,
                  const std::vector<int> &right_line_index);

  void addLSample(int new_sample);
  float getLWeightedMovingAverage();
  float getLMovingAverage();

  void addRSample(int new_sample);
  float getRWeightedMovingAverage();
  float getRMovingAverage();

  
private:
  // Hough Transform Parameters
  enum kHoughIndex { x1 = 0, y1, x2, y2 };
  static const double kHoughRho;
  static const double kHoughTheta;
  static const int kDebgLineWidth = 2;
  static const int kDebugRectangleHalfWidth = 5;
  static const int kDebugRectangleStartHeight = 15;
  static const int kDebugRectangleEndHeight = 25;
  int canny_edge_low_threshold_;
  int canny_edge_high_threshold_;
  float hough_line_slope_range_;
  int hough_threshold_;
  int hough_min_line_length_;
  int hough_max_line_gap_;

  // Image parameters
  int image_width_;
  int image_height_;
  int speed2;

  int pre_left[10] = {0,};
  int pre_right[10] = {640, };
  int left_mean = -1;
  int right_mean = -1;
  int p_lpos = 0;
  int p_rpos = 640;

  const int lSampleSize_ = 10;
  const int rSampleSize_ = 10;
  std::vector<int> l_weight_;
  std::vector<int> r_weight_;
  std::deque<int> l_samples_;
  std::deque<int> r_samples_;

  // line type falg
  static const bool kLeftLane = true;
  static const bool kRightLane = false;

  bool invaild_path_flag;

  // Debug Image and flag
  cv::Mat debug_frame_;
  bool debug_;

};
}  // namespace xycar
#endif  // HOUGH_TRANSFORM_LANE_DETECTOR_H_
