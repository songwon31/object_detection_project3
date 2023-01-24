#include "opencv2/opencv.hpp"
#include <iostream>
#include <random>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <deque>
#include <cmath>

using namespace cv;
using namespace std;

const int image_width_ = 640;
const int image_height_ = 480;
const int roi_start_height_ = 350;
const int roi_height_ = 30;

const float START_SPEED = 0.0F;
const float MAX_SPEED = 15.0F;
const float MIN_SPEED = 10.0F;
const float SPEED_CONTROL_THRESHOLD = 35.0f;
const float ACCELERATION_STEP = 0.01F;
const float DECELERATION_STEP = 0.2F;

const int MOVING_AVERAGE_FILTER = 20;

const int canny_edge_low_threshold_ = 100;
const int canny_edge_high_threshold_ = 200;

const float hough_line_slope_range_ = 20000000.0f;
const int hough_threshold_ = 20;
const int hough_min_line_length_ = 10;
const int hough_max_line_gap_ = 5;

static const int kDebgLineWidth = 2;
static const int kDebugRectangleHalfWidth = 5;
static const int kDebugRectangleStartHeight = 15;
static const int kDebugRectangleEndHeight = 25;

bool kLeftLane = true;
bool kRightLane = false;

const int WIDTH = 640;
const int HEIGHT = 480;
int OFFSET = 385;
const int GAP = 30;

int pre_left[10] = {0, };
int pre_right[10] = {500, };
int left_mean = -1;
int right_mean = -1;

int p_lpos = 0;
int p_rpos = 640;

const int kSampleSize_ = 30;
std::deque<int> samples_;
std::vector<int> weight_;

const int lSampleSize_ = 10;
const int rSampleSize_ = 10;
std::vector<int> l_weight_;
std::vector<int> r_weight_;
std::deque<int> l_samples_;
std::deque<int> r_samples_;

bool invalid_path_flag;

void addSample(int new_sample) {
  samples_.push_back(new_sample);
  if (samples_.size() > kSampleSize_) {
    samples_.pop_front();
  }
}

void addLSample(int new_sample) {
  l_samples_.push_back(new_sample);
  if (l_samples_.size() > lSampleSize_) {
    l_samples_.pop_front();
  }
}

void addRSample(int new_sample) {
  r_samples_.push_back(new_sample);
  if (r_samples_.size() > rSampleSize_) {
    r_samples_.pop_front();
  }
}

float getMovingAverage() {
  int sum = 0, sample_size = samples_.size();
  for (uint8_t i = 0; i < sample_size; ++i) {
    sum += samples_[i];
  }
  return (float)sum / sample_size;
}

float getLMovingAverage() {
  int sum = 0, sample_size = l_samples_.size();
  for (uint8_t i = 0; i < sample_size; ++i) {
    sum += l_samples_[i];
  }
  return (float)sum / sample_size;
}

float getRMovingAverage() {
  int sum = 0, sample_size = r_samples_.size();
  for (uint8_t i = 0; i < sample_size; ++i) {
    sum += r_samples_[i];
  }
  return (float)sum / sample_size;
}

float getWeightedMovingAverage() {
  int sum = 0, weight_sum = 0;
  for (uint8_t i = 0; i < samples_.size(); ++i) {
    sum += samples_[i] * weight_[i];
    weight_sum += weight_[i];
  }
  if (weight_sum == 0) {
    return 0.0f;
  }
  return (float)sum / weight_sum;
}

float getLWeightedMovingAverage() {
  int sum = 0, weight_sum = 0;
  for (uint8_t i = 0; i < l_samples_.size(); ++i) {
    sum += l_samples_[i] * (l_weight_[i] * l_weight_[i]);
    weight_sum += (l_weight_[i] * l_weight_[i]);
  }
  if (weight_sum == 0) {
   throw std::runtime_error("Weight sum is zero");
  }
  return (float)sum / weight_sum;
}

float getRWeightedMovingAverage() {
  int sum = 0, weight_sum = 0;
  for (uint8_t i = 0; i < r_samples_.size(); ++i) {
    sum += r_samples_[i] * (r_weight_[i] * r_weight_[i]);
    weight_sum += (r_weight_[i] * r_weight_[i]);
  }
  if (weight_sum == 0) {
   throw std::runtime_error("Weight sum is zero");
  }
  return (float)sum / weight_sum;
}

void clearSample()
{
  l_samples_.clear();
  r_samples_.clear();
  left_mean = -1;
  right_mean = -1;
}


void draw_lines(
  cv::Mat &image,
  const std::vector<cv::Vec4i> &lines,
  const std::vector<int> &left_line_index,
  const std::vector<int> &right_line_index) {
  cv::Point2i pt1, pt2;
  cv::Scalar color;
  for (int i = 0; i < left_line_index.size(); ++i) {
    pt1 = cv::Point2i(
      lines[left_line_index[i]][0],
      lines[left_line_index[i]][1] + roi_start_height_);
    pt2 = cv::Point2i(
      lines[left_line_index[i]][2],
      lines[left_line_index[i]][3] + roi_start_height_);
    int r, g, b;
    r = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    g = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    b = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    color = std::move(cv::Scalar(b, g, r));
    cv::line(image, pt1, pt2, color, kDebgLineWidth);
  }
  for (int i = 0; i < right_line_index.size(); ++i) {
    pt1 = cv::Point2i(
      lines[right_line_index[i]][0],
      lines[right_line_index[i]][1] + roi_start_height_);
    pt2 = cv::Point2i(
      lines[right_line_index[i]][2],
      lines[right_line_index[i]][3] + roi_start_height_);
    int r, g, b;
    r = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    g = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    b = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    color = std::move(cv::Scalar(b, g, r));
    cv::line(image, pt1, pt2, color, kDebgLineWidth);
  }
}

void draw_rectangles(cv::Mat &image,
                     int lpos,
                     int rpos,
                     int ma_pos) {
  static cv::Scalar kCVRed(0, 0, 255);
  static cv::Scalar kCVGreen(0, 255, 0);
  static cv::Scalar kCVBlue(255, 0, 0);
  cv::rectangle(image,
                cv::Point(lpos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(lpos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVGreen,
                kDebgLineWidth);
  cv::rectangle(image,
                cv::Point(rpos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(rpos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVGreen,
                kDebgLineWidth);
  cv::rectangle(image,
                cv::Point(ma_pos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(ma_pos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVRed,
                kDebgLineWidth);
  cv::rectangle(image,
                cv::Point(image_width_ / 2 - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(image_width_ / 2 + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVBlue,
                kDebgLineWidth);
}

std::pair<float, float> get_line_params(
  const std::vector<cv::Vec4i> &lines, const std::vector<int> &line_index) {
  int lines_size = line_index.size();
  if (lines_size == 0) {
    return std::pair<float, float>(0.0f, 0.0f);
  }

  int x1, y1, x2, y2;
  float x_sum = 0.0f, y_sum = 0.0f, m_sum = 0.0f;
  for (int i = 0; i < lines_size; ++i) {
    x1 = lines[line_index[i]][0],
    y1 = lines[line_index[i]][1];
    x2 = lines[line_index[i]][2],
    y2 = lines[line_index[i]][3];

    x_sum += x1 + x2;
    y_sum += y1 + y2;

    if (x2 == x1) {
      if (y1 > y2) {
            m_sum += -30.0f;
          } else {
            m_sum += 30.0f;
          }
    } else {
      m_sum += (float)(y2 - y1) / (float)(x2 - x1);
    }
  }

  float x_avg, y_avg, m, b;
  x_avg = x_sum / (lines_size * 2);
  y_avg = y_sum / (lines_size * 2);
  m = m_sum / lines_size;
  b = y_avg - m * x_avg;

  std::pair<float, float> m_and_b(m, b);
  return m_and_b;
}

int get_line_pos(
  const std::vector<cv::Vec4i> &lines,
  const std::vector<int> &line_index,
  const bool direction) {
  float m, b;
  std::tie(m, b) = get_line_params(lines, line_index);

  float y, pos;
  if (m == 0.0 && b == 0.0) {
    if (direction == kLeftLane) {
      if (invalid_path_flag)
      {
        pos = left_mean;
      }
      else
      {
        pos = 0.0f;
      }
    } else {
      if (invalid_path_flag)
      {
        pos = right_mean;
      }
      else
      {
        pos = (float)image_width_;
      }
    }
  } else {
    y = (float)roi_height_ * 0.5;
    pos = (y - b) / m;
  }
  return std::round(pos);
}

std::pair<std::vector<int>, std::vector<int>>
    divideLines(const std::vector<cv::Vec4i> &lines) {
    int lines_size = lines.size();
    std::vector<int> left_line_index;
    std::vector<int> right_line_index;
    left_line_index.reserve(lines_size);
    right_line_index.reserve(lines_size);
    int x1, y1, x2, y2;
    float slope;
    float left_line_x_sum = 0.0f;
    float right_line_x_sum = 0.0f;
    float left_x_avg, right_x_avg;

    int max_left_x1;
    int max_left_x2 = 0;
    int max_left_index = -1;
    int min_right_x2;
    int min_right_x1 = 640;
    int min_right_index = -1;

    for (int i = 0; i < lines_size; ++i) {
      x1 = lines[i][0], y1 = lines[i][1];
      x2 = lines[i][2], y2 = lines[i][3];
      if (x2 - x1 == 0) {
        if (y1 > y2) {
          slope = -30.0f;
        } else {
          slope = 30.0f;
        }
      } else {
          slope = (float)(y2 - y1) / (float)(x2 - x1);
      }

      int x_mean = (x1+x2)/2;

      if (left_mean == -1) 
      {
        if (slope < 0 && x2 < 320) {
          if (x2 > max_left_x2) {
            max_left_x2 = x2;
            max_left_index = i;
            max_left_x1 = x1;
          }
        } else if (slope > 0 && x1 > 320) {
          if (x1 < min_right_x1) {
            min_right_x1 = x1;
            min_right_index = i;
            min_right_x2=x2;
          }
        }
      }
      else
      {
        if (slope < 0 && std::abs(left_mean - x_mean) < 50) {
          left_line_x_sum += (float)(x1 + x2) * 0.5;
          left_line_index.push_back(i);
        } else if (0 < slope && std::abs(right_mean - x_mean) < 50) {
          right_line_x_sum += (float)(x1 + x2) * 0.5;
          right_line_index.push_back(i);
        }
        
        /*
        if (((slope < 0 && x2 < 540) || (slope > 0 && x2 < 450)) &&
            (left_mean == -2 || (left_mean != -1 && std::abs(left_mean - x_mean) < 80) || (left_mean == -1 && x2 < 320))) {
            left_line_x_sum += (float)(x1 + x2) * 0.5;
            left_line_index.push_back(i);
        } else if (((0 < slope && x1 > 90) || (slope < 0 && x1>290)) &&
            (right_mean == -2 || (right_mean != -1 && std::abs(right_mean - x_mean) < 80) || (right_mean == -1 && x1 > 320))) {
            right_line_x_sum += (float)(x1 + x2) * 0.5;
            right_line_index.push_back(i);
        }
        */
       /*
        if (
            (left_mean == -2 || (left_mean != -1 && std::abs(left_mean - x_mean) < 80) || (left_mean == -1 && x2 < 320))) {
            left_line_x_sum += (float)(x1 + x2) * 0.5;
            left_line_index.push_back(i);
        } else if (
            (right_mean == -2 || (right_mean != -1 && std::abs(right_mean - x_mean) < 80) || (right_mean == -1 && x1 > 320))) {
            right_line_x_sum += (float)(x1 + x2) * 0.5;
            right_line_index.push_back(i);
        }
        */
      }
    }


    /*
    cout << '\n';
    cout << "all_lines: [";
    for (int i = 0; i < lines.size(); ++i) {
      cout << '[';
      for (int j = 0; j < 4; ++j) {
        if (j==3) {
          cout << lines[i][j];
        } else {
          cout << lines[i][j] << ',';
        }
      }
      cout << ']';
      if (i < lines.size()-1) {
        cout << ' ';
      } 
    }
    cout <<'\n';

    cout << "left_lines: [";
    for (int i = 0; i < left_line_index.size(); ++i) {
      cout << '[';
      for (int j = 0; j < 4; ++j) {
        if (j==3) {
          cout << lines[left_line_index[i]][j];
        } else {
          cout << lines[left_line_index[i]][j] << ',';
        }
      }
      cout << ']';
      if (i < left_line_index.size()-1) {
        cout << ' ';
      }
    }
    cout <<'\n';
    

    cout << "right_lines: [";
    for (int i = 0; i < right_line_index.size(); ++i) {
      cout << '[';
      for (int j = 0; j < 4; ++j) {
        if (j==3) {
          cout << lines[right_line_index[i]][j];
        } else {
          cout << lines[right_line_index[i]][j] << ',';
        }
      }
      cout << ']';
      if (i < right_line_index.size()-1) {
        cout << ' ';
      }
    }
    cout <<'\n';
    */

    if (left_mean == -1) {
      left_line_x_sum += (float)(max_left_x1 + max_left_x2) * 0.5;
      left_line_index.push_back(max_left_index);
      right_line_x_sum += (float)(min_right_x1 + min_right_x2) * 0.5;
      right_line_index.push_back(min_right_index);
    }

    int left_lines_size = left_line_index.size();
    int right_lines_size = right_line_index.size();
    invalid_path_flag = false;
    if (left_lines_size != 0 && right_lines_size != 0) {
        left_x_avg = left_line_x_sum / left_lines_size;
        right_x_avg = right_line_x_sum / right_lines_size;
        if (left_x_avg > right_x_avg) {
            left_line_index.clear();
            right_line_index.clear();
            std::cout << "------Invalid Path!------\n";
            invalid_path_flag = true;
        }
    }
    

    return std::pair<std::vector<int>, std::vector<int>>(
    std::move(left_line_index), std::move(right_line_index));
}

std::pair<int, int> getLanePosition(cv::Mat &image) {
    // gray
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat blur_image;
    cv::GaussianBlur(gray_image, blur_image, cv::Size(5, 5), 1.5);

    cv::Mat canny_image;
    cv::Canny(blur_image,
              canny_image,
              canny_edge_low_threshold_,
              canny_edge_high_threshold_);

    cv::Mat roi = canny_image(cv::Rect(0, roi_start_height_, image_width_, roi_height_));

    cv::dilate(roi, roi, cv::Mat());
    imshow("dilate", roi);

    std::vector<cv::Vec4i> all_lines;
    cv::HoughLinesP(roi,
                    all_lines,
                    1,
                    CV_PI/180,
                    hough_threshold_,
                    hough_min_line_length_,
                    hough_max_line_gap_);

    if (all_lines.size() == 0) {
      clearSample();
      return std::pair<int, int>(p_lpos,p_rpos);
    }

    std::vector<int> left_line_index, right_line_index;
    std::tie(left_line_index, right_line_index) = std::move(divideLines(all_lines));

    int lpos = get_line_pos(all_lines, left_line_index, kLeftLane);
    int rpos = get_line_pos(all_lines, right_line_index, kRightLane);

    draw_lines(image, all_lines, left_line_index, right_line_index);

    return std::pair<int, int>(lpos, rpos);
}

void start() 
{
    VideoCapture cap;
    cap.open("label_img3.avi");
    
    if (!cap.isOpened())
    {
        cerr << "Video open failed!\n";
        exit(1);
    }

    Mat frame_;
    namedWindow("frame");

    while (true)    
    {
        if (!cap.read(frame_)) 
        {
            cout << "Video end!\n";
            break;
        }

        int lpos, rpos, error, ma_mpos;

        std::tie(lpos, rpos) = getLanePosition(frame_);

        addSample((lpos+rpos)/2);
        ma_mpos = getWeightedMovingAverage();

        std::cout << "lpos: " << lpos << ", rpos: " << rpos << ", mpos: " << ma_mpos << std::endl;
        draw_rectangles(frame_, lpos, rpos, ma_mpos);
        imshow("frame", frame_);
        cout << '\n';

        if (lpos == 0 && rpos == image_width_) {
          clearSample();
          lpos = p_lpos;
          rpos = p_rpos;
        } else {
          addLSample(lpos);
          left_mean = getLWeightedMovingAverage();
          addRSample(rpos);
          right_mean = getRWeightedMovingAverage();
        }

        p_lpos = lpos;
        p_rpos = rpos;

        waitKey();
    }
}

int main(int argc, char** argv) 
{
    weight_.reserve(kSampleSize_);
    for (uint8_t i = 1; i<=kSampleSize_; ++i) {
        weight_.push_back(i);
    }

    l_weight_.reserve(lSampleSize_);
    r_weight_.reserve(rSampleSize_);
    for (uint8_t i = 1; i<=lSampleSize_; ++i) {
      l_weight_.push_back(i);
      r_weight_.push_back(i);
    }

    start();
    return 0;
}