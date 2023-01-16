#ifndef LANE_KEEPING_SYSTEM_H_
#define LANE_KEEPING_SYSTEM_H_

#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <xycar_msgs/xycar_motor.h>
#include <yaml-cpp/yaml.h>

#include "lane_keeping_system/hough_transform_lane_detector.h"
#include "lane_keeping_system/moving_average_filter.h"
#include "lane_keeping_system/pid_controller.h"

#include "yolov3_trt_ros/BoundingBox.h"
#include "yolov3_trt_ros/BoundingBoxes.h"

#include <unistd.h>

namespace xycar
{
class LaneKeepingSystem
{
public:
  // Construct a new Lane Keeping System object
  LaneKeepingSystem();
  // Destroy the Lane Keeping System object
  virtual ~LaneKeepingSystem();
  // Running Lane Keeping System Algorithm
  void run();
  // Set parameters from config file
  void setParams(const YAML::Node& config);

private:
  // Setting steering angle and speed
  void speed_control(float steering_angle);

  // Puslish Xycar Moter Message
  void drive(float steering_angle);

  // Subcribe Image Topic
  void imageCallback(const sensor_msgs::Image& msg);

  std::fstream outfile;

private:
  // Xycar Steering Angle Limit
  static const int kXycarSteeringAngleLimit = 50;
  // PID Class for Control
  PID* pid_ptr_;
  // Moving Average Filter Class for Noise filtering
  MovingAverageFilter* ma_filter_ptr_;
  // Hough Transform Lane Detector Class for Lane Detection
  HoughTransformLaneDetector* hough_transform_lane_detector_ptr_;

  // ROS Variables
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber object_sub_;
  std::string pub_topic_name_;
  std::string sub_topic_name_;
  uint16_t queue_size_;
  xycar_msgs::xycar_motor msg_;

  // OpenCV Image processing Variables
  cv::Mat frame_;

  // Xycar Device variables
  float xycar_speed_;
  float xycar_max_speed_;
  float xycar_min_speed_;
  float xycar_speed_control_threshold_;
  float acceleration_step_;
  float deceleration_step_;

  // Debug Flag
  bool debug_;
};
}  // namespace xycar

#endif  // LANE_KEEPING_SYSTEM_H_
