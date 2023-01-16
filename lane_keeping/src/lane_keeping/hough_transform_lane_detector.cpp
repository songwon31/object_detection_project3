#include "lane_keeping_system/hough_transform_lane_detector.h"

namespace xycar
{
const double HoughTransformLaneDetector::kHoughRho = 1.0;
const double HoughTransformLaneDetector::kHoughTheta = CV_PI / 180.0;

HoughTransformLaneDetector::HoughTransformLaneDetector(const YAML::Node& config)
{
  set(config);
}

void HoughTransformLaneDetector::set(const YAML::Node& config)
{
  image_width_ = config["IMAGE"]["WIDTH"].as<uint16_t>();
  image_height_ = config["IMAGE"]["HEIGHT"].as<uint16_t>();
  roi_start_height_ = config["IMAGE"]["ROI_START_HEIGHT"].as<uint16_t>();
  roi_height_ = config["IMAGE"]["ROI_HEIGHT"].as<uint8_t>();
  canny_edge_low_threshold_ = config["CANNY"]["LOW_THRESHOLD"].as<uint16_t>();
  canny_edge_high_threshold_ = config["CANNY"]["HIGH_THRESHOLD"].as<uint16_t>();
  hough_line_slope_range_ = config["HOUGH"]["ABS_SLOPE_RANGE"].as<uint16_t>();
  hough_threshold_ = config["HOUGH"]["THRESHOLD"].as<uint8_t>();
  hough_min_line_length_ = config["HOUGH"]["MIN_LINE_LENGTH"].as<uint8_t>();
  hough_max_line_gap_ = config["HOUGH"]["MAX_LINE_GAP"].as<uint8_t>();
  debug_ = config["DEBUG"].as<bool>();

  left_mean = -1;
  right_mean = -1;

  l_weight_.reserve(lSampleSize_);
  for (uint16_t i = 1; i <= lSampleSize_; ++i)
  {
    l_weight_.push_back(i * i);
  }

  r_weight_.reserve(rSampleSize_);
  for (uint16_t i = 1; i <= rSampleSize_; ++i)
  {
    r_weight_.push_back(i * i);
  }
}

void HoughTransformLaneDetector::getSpeed(uint8_t speed)
{
  current_speed = speed;
}

std::pair<float, float> HoughTransformLaneDetector::get_line_params(const std::vector<cv::Vec4i>& lines,
                                                                    const std::vector<uint16_t>& line_index)
{
  uint32_t lines_size = line_index.size();
  if (lines_size == 0)
  {
    return std::pair<float, float>(0.0f, 0.0f);
  }

  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;
  float x_sum = 0.0f;
  float y_sum = 0.0f;
  float slope_sum = 0.0f;
  for (uint32_t i = 0; i < lines_size; ++i)
  {
    x1 = lines[line_index[i]][kHoughIndex::x1], y1 = lines[line_index[i]][kHoughIndex::y1];
    x2 = lines[line_index[i]][kHoughIndex::x2], y2 = lines[line_index[i]][kHoughIndex::y2];

    x_sum += x1 + x2;
    y_sum += y1 + y2;

    if (x2 == x1)
    {
      if (y1 > y2)
      {
        slope_sum += -30.0f;
      }
      else
      {
        slope_sum += 30.0f;
      }
    }
    else
    {
      slope_sum += (float)(y2 - y1) / (float)(x2 - x1);
    }
  }

  float x_avg;
  float y_avg;
  float slope;
  float y_intercept;
  x_avg = x_sum / (lines_size * 2);
  y_avg = y_sum / (lines_size * 2);
  slope = slope_sum / lines_size;
  y_intercept = y_avg - slope * x_avg;

  std::pair<float, float> slope_and_y_intercept(slope, y_intercept);
  return slope_and_y_intercept;
}

uint16_t HoughTransformLaneDetector::get_line_pos(const std::vector<cv::Vec4i>& lines,
                                                  const std::vector<uint16_t>& line_index, const bool direction)
{
  float slope;
  float y_intercept;
  std::tie(slope, y_intercept) = get_line_params(lines, line_index);

  float y;
  float pos;
  if (slope == 0.0 && y_intercept == 0.0)
  {
    if (direction == kLeftLane)
    {
      if (invaild_path_flag)
      {
        pos = left_mean;
      }
      else
      {
        pos = 0.0f;
      }
    }
    else
    {
      if (invaild_path_flag)
      {
        pos = right_mean;
      }
      else
      {
        pos = 640.0f;
      }
    }
  }
  else
  {
    y = (float)roi_height_ * 0.5;
    pos = (y - y_intercept) / slope;
  }
  return (uint16_t)std::round(pos);
}

std::pair<std::vector<uint16_t>, std::vector<uint16_t>>
HoughTransformLaneDetector::divideLines(const std::vector<cv::Vec4i>& lines)
{
  uint32_t lines_size = lines.size();
  std::vector<uint16_t> left_line_index;
  std::vector<uint16_t> right_line_index;
  left_line_index.reserve(lines_size);
  right_line_index.reserve(lines_size);
  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;
  float slope;
  float left_line_x_sum = 0.0f;
  float right_line_x_sum = 0.0f;
  float left_x_avg;
  float right_x_avg;

  uint16_t max_left_x1;
  uint16_t max_left_x2 = 0;
  uint16_t max_left_index = -1;
  uint16_t min_right_x2;
  int min_right_x1 = 640;
  int min_right_index = -1;

  for (uint32_t i = 0; i < lines_size; ++i)
  {
    x1 = lines[i][kHoughIndex::x1], y1 = lines[i][kHoughIndex::y1];
    x2 = lines[i][kHoughIndex::x2], y2 = lines[i][kHoughIndex::y2];
    if (x2 - x1 == 0)
    {
      if (y1 > y2)
      {
        slope = -30.0f;
      }
      else
      {
        slope = 30.0f;
      }
    }
    else
    {
      slope = (float)(y2 - y1) / (float)(x2 - x1);
    }

    int x_mean = (x1 + x2) / 2;

    if (left_mean == -1)
    {
      if (slope < 0 && x2 < 320)
      {
        if (x2 > max_left_x2)
        {
          max_left_x2 = x2;
          max_left_index = i;
          max_left_x1 = x1;
        }
      }
      else if (slope > 0 && x1 > 320)
      {
        if (x1 < min_right_x1)
        {
          min_right_x1 = x1;
          min_right_index = i;
          min_right_x2 = x2;
        }
      }
    }
    else
    {
      if (slope < 0 && (left_mean == -2 || (left_mean != -1 && std::abs(left_mean - x_mean) < 80)))
      {
        left_line_x_sum += (float)(x1 + x2) * 0.5;
        left_line_index.push_back(i);
      }
      else if (0 < slope && (right_mean == -2 || (right_mean != -1 && std::abs(right_mean - x_mean) < 80)))
      {
        right_line_x_sum += (float)(x1 + x2) * 0.5;
        right_line_index.push_back(i);
      }
    }
  }

  if (left_mean == -1)
  {
    left_line_x_sum += (float)(max_left_x1 + max_left_x2) * 0.5;
    left_line_index.push_back(max_left_index);
    right_line_x_sum += (float)(min_right_x1 + min_right_x2) * 0.5;
    right_line_index.push_back(min_right_index);
  }

  int left_lines_size = left_line_index.size();
  int right_lines_size = right_line_index.size();
  invaild_path_flag = false;
  if (left_lines_size != 0 && right_lines_size != 0)
  {
    left_x_avg = left_line_x_sum / left_lines_size;
    right_x_avg = right_line_x_sum / right_lines_size;
    if (left_x_avg > right_x_avg)
    {
      left_line_index.clear();
      right_line_index.clear();
      // std::cout << "------Invalid Path!------\n";
      invaild_path_flag = true;
    }
  }

  return std::pair<std::vector<uint16_t>, std::vector<uint16_t>>(std::move(left_line_index),
                                                                 std::move(right_line_index));
}

std::pair<uint16_t, uint16_t> HoughTransformLaneDetector::getLanePosition(const cv::Mat& image)
{
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  cv::Mat blur_image;
  cv::GaussianBlur(gray_image, blur_image, cv::Size(5, 5), 1.5);
  cv::Mat canny_image;
  cv::Canny(blur_image, canny_image, canny_edge_low_threshold_, canny_edge_high_threshold_);
  if (current_speed > 22)
  {
    roi_start_height_ = 340;
  }
  else if (current_speed > 20)
  {
    roi_start_height_ = 345;
  }
  else
  {
    roi_start_height_ = 360;
  }
  cv::Mat roi = canny_image(cv::Rect(0, roi_start_height_, image_width_, roi_height_));
  cv::dilate(roi, roi, cv::Mat());
  cv::Mat mask_img = cv::imread("mask.png", cv::IMREAD_GRAYSCALE);
  cv::bitwise_and(roi, mask_img, roi);
  std::vector<cv::Vec4i> all_lines;
  cv::HoughLinesP(roi, all_lines, kHoughRho, kHoughTheta, hough_threshold_, hough_min_line_length_,
                  hough_max_line_gap_);
  if (all_lines.size() == 0)
  {
    return std::pair<uint16_t, uint16_t>(0, image_width_);
  }

  std::vector<uint16_t> left_line_index;
  std::vector<uint16_t> right_line_index;
  std::tie(left_line_index, right_line_index) = std::move(divideLines(all_lines));

  uint16_t lpos = get_line_pos(all_lines, left_line_index, kLeftLane);
  uint16_t rpos = get_line_pos(all_lines, right_line_index, kRightLane);

  if (lpos == 0 && rpos == image_width_)
  {
    l_samples_.clear();
    r_samples_.clear();
    left_mean = -2;
    right_mean = -2;
  }
  else
  {
    addLSample(lpos);
    left_mean = getLWeightedMovingAverage();
    addRSample(rpos);
    right_mean = getRWeightedMovingAverage();
  }

  return std::pair<uint16_t, uint16_t>(lpos, rpos);
}

void HoughTransformLaneDetector::addLSample(uint16_t new_sample)
{
  l_samples_.push_back(new_sample);
  if (l_samples_.size() > lSampleSize_)
  {
    l_samples_.pop_front();
  }
}

void HoughTransformLaneDetector::addRSample(uint16_t new_sample)
{
  r_samples_.push_back(new_sample);
  if (r_samples_.size() > rSampleSize_)
  {
    r_samples_.pop_front();
  }
}

float HoughTransformLaneDetector::getLWeightedMovingAverage()
{
  uint32_t sum = 0, weight_sum = 0;
  for (uint32_t i = 0; i < l_samples_.size(); ++i)
  {
    sum += l_samples_[i] * l_weight_[i];
    weight_sum += l_weight_[i];
  }
  if (weight_sum == 0)
  {
    return 0.0f;
  }
  return (float)sum / weight_sum;
}

float HoughTransformLaneDetector::getRWeightedMovingAverage()
{
  uint32_t sum = 0, weight_sum = 0;
  for (uint32_t i = 0; i < r_samples_.size(); ++i)
  {
    sum += r_samples_[i] * r_weight_[i];
    weight_sum += r_weight_[i];
  }
  if (weight_sum == 0)
  {
    throw std::runtime_error("RWeight sum is zero");
  }
  return (float)sum / weight_sum;
}

}  // namespace xycar
