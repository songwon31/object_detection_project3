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
  ma_filter_ptr_ = new MovingAverageFilter(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<int>());
  hough_transform_lane_detector_ptr_ = new HoughTransformLaneDetector(config);
  setParams(config);
  pub_ = nh_.advertise<xycar_msgs::xycar_motor>(pub_topic_name_, queue_size_);
  image_sub_ = nh_.subscribe(sub_topic_name_, queue_size_, &LaneKeepingSystem::imageCallback, this);
  detection_sub_ = nh_.subscribe("/yolov3_trt_ros/detections", queue_size_, &LaneKeepingSystem::detectionCallback, this);
}

void LaneKeepingSystem::setParams(const YAML::Node &config) {
  pub_topic_name_ = config["TOPIC"]["PUB_NAME"].as<std::string>();
  sub_topic_name_ = config["TOPIC"]["SUB_NAME"].as<std::string>();
  queue_size_ = config["TOPIC"]["QUEUE_SIZE"].as<int>();

  xycar_speed_ = config["XYCAR"]["START_SPEED"].as<float>();
  xycar_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<float>();
  xycar_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<float>();
  xycar_speed_control_threshold_ =
    config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
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
  while (detection_sub_.getNumPublishers() == 0)
  {}
  int lpos, rpos, error, ma_mpos, left_mpos, right_mpos;
  float steering_angle;
  ros::Rate rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    if (frame_.empty())
    {
      continue;
    }

    if (object_id >= 0)
    {
      if (object_id == 0)
      {
        drive_left_or_right("left", 2.5f);
      }
      else if (object_id == 1)
      {
        drive_left_or_right("right", 2.5f);
      }
      else if (object_id == 2 || object_id == 3)
      {
        drive_stop(6.0f);
      }
      else if (object_id == 4)
      {
        detect_traffic_light(6.0f);
      }
    }
    else
    {
      drive_normal();
    }
    object_id = -1;
  }
}

void LaneKeepingSystem::imageCallback(const sensor_msgs::Image& msg)
{
  cv::Mat src = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t*>(&msg.data[0]), msg.step);
  cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void LaneKeepingSystem::detectionCallback(const yolov3_trt_ros::BoundingBoxes& msg) {
  // TODO: Implement your code here
  //* EXAMPLE CODE
  for (auto& box : msg.bounding_boxes) {
    std::cout << "Class ID: " << box.id << std::endl;
    object_id = box.id;
    box_xmin = box.xmin;
    box_ymin = box.ymin;
    box_xmax = box.xmax;
    box_ymax = box.ymax;
  }
}

void LaneKeepingSystem::drive_normal()
{
  int lpos, rpos, error, ma_mpos, left_mpos, right_mpos;
  float steering_angle;

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

void LaneKeepingSystem::drive_left_or_right(std::string direction, float time) {
  ros::spinOnce();
  // Set the rate variable
  ros::Rate rate(sleep_rate);

  int lpos, rpos, error, ma_mpos, left_mpos, right_mpos;
  float steering_angle;
  float max_cnt;
  int cnt = 0;

  if (time == 0) {  // time 0 means straight drive.
    max_cnt = 1;
  } else {
    // Set the maximun number of iteration in while loop.
    max_cnt = static_cast<float>(sleep_rate) * time;
  }

  while (static_cast<float>(cnt) < max_cnt) {
    int lpos, rpos;
    std::tie(lpos, rpos) = hough_transform_lane_detector_ptr_->getLanePosition(frame_);

    // Left or Right
    if (direction == "left") {
      rpos = lpos + 470;
    } else if (direction == "right") {
      lpos = rpos - 470;
    }

    ma_filter_ptr_->addSample((lpos + rpos) / 2);
    ma_mpos = ma_filter_ptr_->getWeightedMovingAverage();
    error = ma_mpos - frame_.cols / 2;
    steering_angle = std::max(-(float)kXycarSteeringAngleLimit,
                              std::min(pid_ptr_->getControlOutput(error), (float)kXycarSteeringAngleLimit));

    pid_ptr_->getAngle(steering_angle);

    speed_control(steering_angle);
    drive(steering_angle);

    cnt++;
    rate.sleep();
  }
}

void LaneKeepingSystem::drive_stop(float time)
{
  ros::spinOnce();
  ros::Rate rate(sleep_rate);

  float max_cnt;
  int cnt = 0;

  if (time == 0) {
    max_cnt = 1;
  } else {
    max_cnt = static_cast<float>(sleep_rate) * time;
  }

  while (static_cast<float>(cnt) < max_cnt) {
    xycar_msgs::xycar_motor motor_msg;
    motor_msg.angle = 0;
    motor_msg.speed = 0;
    pub_.publish(motor_msg);

    cnt++;
    rate.sleep();
  }

  max_cnt = static_cast<float>(sleep_rate) * 2.0f;
  cnt = 0;
  while (static_cast<float>(cnt) < max_cnt) {
    drive_normal();
    cnt++;
    rate.sleep();
  }

}

void LaneKeepingSystem::detect_traffic_light(float time)
{
  cv::Mat traffic_light = frame_(cv::Range(box_xmin, box_xmax), cv::Range(box_ymin, box_ymax));

  // cv::Mat traffic_light_upside = frame_(cv::Range(box_xmin, box_xmax), cv::Range(box_ymin, box_ymin + (box_ymax -
  // box_ymin) / 3)); cv::Mat traffic_light_downside = frame_(cv::Range(box_xmin, box_xmax), cv::Range(box_ymin +
  // (box_ymax - box_ymin) *  2 / 3, box_ymax));

  // cv::Mat hsv_upside, hsv_downside;
  // cv::cvtColor(traffic_light_upside, hsv_upside, cv::COLOR_BGR2HSV);

  cv::Mat hsv;
  cv::cvtColor(traffic_light, hsv, cv::COLOR_BGR2HSV);

  std::vector<cv::Mat> planes;
  cv::split(hsv, planes);

  cv::Mat h= planes[0];
  cv::Mat v = planes[2];

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(v, circles, cv::HOUGH_GRADIENT_ALT, 1.5, 10, 300, 0.9, 20, 50);

  int max_idx(-1);
  float max_val(0);
  cv::Rect max_rect;

  for (int i = 0; i < circles.size(); i++)
  {
    cv::Vec3i c = circles[i];

    cv::Rect rect(c[0]-c[2], c[1]-c[2], c[2]*2, c[2]*2);

    float mean_v = cv::mean(v(rect))[0];
    if (mean_v > max_val)
    {
      max_idx = i;
      max_val = mean_v;
      max_rect = rect;
    }
  }

  h += 30;

  float mean_hue = cv::mean(h(max_rect))[0];
  if(70 < mean_hue && mean_hue < 150)
  {
    drive_normal();
  }
  else if(0 < mean_hue && mean_hue < 60 )
  {
    xycar_msgs::xycar_motor motor_msg;
    motor_msg.angle = 0;
    motor_msg.speed = 0;
    pub_.publish(motor_msg);
  }
  else 
  {
    drive_normal();
  }
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
