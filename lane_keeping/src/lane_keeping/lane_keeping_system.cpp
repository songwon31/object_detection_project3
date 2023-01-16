#include "lane_keeping_system/lane_keeping_system.h"

namespace xycar
{
LaneKeepingSystem::LaneKeepingSystem()
{
  std::string config_path;
  nh_.getParam("config_path", config_path);
  YAML::Node config = YAML::LoadFile(config_path);
  pid_ptr_ = new PID(config["PID"]["P_GAIN"].as<float>(), config["PID"]["I_GAIN"].as<float>(),
                     config["PID"]["D_GAIN"].as<float>());
  ma_filter_ptr_ = new MovingAverageFilter(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint8_t>());
  hough_transform_lane_detector_ptr_ = new HoughTransformLaneDetector(config);
  setParams(config);
  pub_ = nh_.advertise<xycar_msgs::xycar_motor>(pub_topic_name_, queue_size_);
  sub_ = nh_.subscribe(sub_topic_name_, queue_size_, &LaneKeepingSystem::imageCallback, this);
}

void LaneKeepingSystem::setParams(const YAML::Node& config)
{
  pub_topic_name_ = config["TOPIC"]["PUB_NAME"].as<std::string>();
  sub_topic_name_ = config["TOPIC"]["SUB_NAME"].as<std::string>();
  queue_size_ = config["TOPIC"]["QUEUE_SIZE"].as<int>();

  xycar_speed_ = config["XYCAR"]["START_SPEED"].as<float>();
  xycar_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<float>();
  xycar_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<float>();
  xycar_speed_control_threshold_ = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
  acceleration_step_ = config["XYCAR"]["ACCELERATION_STEP"].as<float>();
  deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<float>();
  debug_ = config["DEBUG"].as<bool>();
}

LaneKeepingSystem::~LaneKeepingSystem()
{
  delete ma_filter_ptr_;
  delete pid_ptr_;
  delete hough_transform_lane_detector_ptr_;
}

void LaneKeepingSystem::run()
{
  uint16_t lpos;
  uint16_t rpos;
  uint16_t ma_mpos;
  int32_t error;
  float steering_angle;
  ros::Rate rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    if (frame_.empty())
    {
      continue;
    }

    std::tie(lpos, rpos) = hough_transform_lane_detector_ptr_->getLanePosition(frame_);

    ma_filter_ptr_->addSample((lpos + rpos) / 2);
    ma_mpos = ma_filter_ptr_->getWeightedMovingAverage();
    error = ma_mpos - frame_.cols / 2;
    steering_angle = std::max(-(float)kXycarSteeringAngleLimit,
                              std::min(pid_ptr_->getControlOutput(error), (float)kXycarSteeringAngleLimit));

    pid_ptr_->getAngle(steering_angle);

    speed_control(steering_angle);
    drive(steering_angle);
  }
}

void LaneKeepingSystem::imageCallback(const sensor_msgs::Image& msg)
{
  cv::Mat src = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t*>(&msg.data[0]), msg.step);
  cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void LaneKeepingSystem::speed_control(float steering_angle)
{
  float yaw = abs(steering_angle);
  if (yaw > xycar_speed_control_threshold_)
  {
    xycar_speed_ -= deceleration_step_ * ((exp(yaw / 20 - 0.5)) - pow(1.3, yaw / 20 - 0.5));
    xycar_speed_ = std::max(xycar_speed_, xycar_min_speed_);
  }
  else
  {
    xycar_speed_ += acceleration_step_ * exp(yaw / 100) * 0.6;
    xycar_speed_ = std::min(xycar_speed_, xycar_max_speed_);
  }
  hough_transform_lane_detector_ptr_->getSpeed(xycar_speed_);
}

void LaneKeepingSystem::drive(float steering_angle)
{
  xycar_msgs::xycar_motor motor_msg;
  if (abs(steering_angle) < 10)
  {
    steering_angle = 0;
  }
  motor_msg.angle = std::round(steering_angle);
  motor_msg.speed = std::round(xycar_speed_);
  pub_.publish(motor_msg);
}
}  // namespace xycar
