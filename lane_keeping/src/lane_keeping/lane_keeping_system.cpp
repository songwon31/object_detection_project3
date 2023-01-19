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
  int lpos, rpos, error, ma_mpos;
  float steering_angle;
  ros::Rate rate(sleep_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    if (frame_.empty())
    {
      continue;
    }

    std::tie(lpos, rpos) = hough_transform_lane_detector_ptr_->getLanePosition(frame_);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (object_id == 0)
    {
      rpos = std::min(lpos + 470, 640);
    }
    else if (object_id == 1)
    {
      lpos = std::max(0, rpos - 470);
    }
    else if (object_id == 2 || object_id == 3)
    {
      drive_stop(6.0f);
      continue;
    }
    else if (object_id == 4)
    { 
      traffic_sign_recognition();
      continue;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (lpos != 0 && rpos == 640)
    {
      rpos = std::min(lpos + 470, 640);
    }
    else if (lpos ==0 && rpos != 640)
    {
      lpos = std::max(0, rpos - 470);
    }

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

void LaneKeepingSystem::detectionCallback(const yolov3_trt_ros::BoundingBoxes& msg) {
  // TODO: Implement your code here
  //* EXAMPLE CODE
  if (msg.bounding_boxes.size() > 0)
  {
    std::vector<yolov3_trt_ros::BoundingBox> all_signs;
    for (auto& box : msg.bounding_boxes) {
      all_signs.push_back(box);
      /*
      std::cout << "Class ID: " << box.id << std::endl;
      object_id = box.id;
      box_xmin = box.xmin;
      box_ymin = box.ymin;
      box_xmax = box.xmax;
      box_ymax = box.ymax;
      */
    }

    int focus_index = 0;
    int biggest_sign_size = (all_signs[0].xmax - all_signs[0].xmin) * (all_signs[0].ymax - all_signs[0].ymin);
    if (all_signs[0].id == 4)
    {
      biggest_sign_size /= 2;
    }
    for (int i=1; i<all_signs.size(); ++i)
    {
      int cur_size = (all_signs[i].xmax - all_signs[i].xmin) * (all_signs[i].ymax - all_signs[i].ymin);
      if (all_signs[0].id == 4)
      {
        biggest_sign_size /= 2;
      }
      if (biggest_sign_size < cur_size)
      {
        biggest_sign_size = cur_size;
        focus_index = i;
      }
    }

    std::cout << "Class ID: " << all_signs[focus_index].id << std::endl;
    std::cout << (all_signs[focus_index].xmax - all_signs[focus_index].xmin) * (all_signs[focus_index].ymax - all_signs[focus_index].ymin) << std::endl;
    object_id = all_signs[focus_index].id;
    box_xmin = all_signs[focus_index].xmin;
    box_ymin = all_signs[focus_index].ymin;
    box_xmax = all_signs[focus_index].xmax;
    box_ymax = all_signs[focus_index].ymax;
  }
  else
  {
    object_id = -1;
  }
}

void LaneKeepingSystem::drive_normal()
{
  int lpos, rpos, error, ma_mpos, left_mpos, right_mpos;
  float steering_angle;

  std::tie(lpos, rpos) = hough_transform_lane_detector_ptr_->getLanePosition(frame_);

  if (lpos != 0 && rpos == 640)
  {
    rpos = lpos + 470;
  } 
  else if (lpos == 0 && rpos != 640)
  {
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
}

void LaneKeepingSystem::drive_left_or_right(std::string direction, float time) {
  ros::spinOnce();
  // Set the rate variable
  ros::Rate rate(sleep_rate);

  int lpos, rpos, error, ma_mpos;
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
    std::cout << lpos << ' ' << rpos << '\n';

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

  if (past_steering_angle < -10)
  {
    cv::Mat stop_line = frame_(cv::Rect(350, 350, 20, 40));
    cv::cvtColor(stop_line, stop_line, cv::COLOR_BGR2GRAY);
    cv::threshold(stop_line, stop_line, 200, 255, cv::THRESH_BINARY);
    std::cout << (int)cv::mean(stop_line)[0] << std::endl;
    if ((int)cv::mean(stop_line)[0] > 230) {
      std::cout << 1 << std::endl;
      drive_normal();
      return;
    }
  } 
  else if (past_steering_angle > 10) 
  {
    cv::Mat stop_line = frame_(cv::Rect(270, 350, 20, 40));
    cv::cvtColor(stop_line, stop_line, cv::COLOR_BGR2GRAY);
    cv::threshold(stop_line, stop_line, 200, 255, cv::THRESH_BINARY);
    std::cout << (int)cv::mean(stop_line)[0] << std::endl;
    if ((int)cv::mean(stop_line)[0] > 230) {
      std::cout << 1 << std::endl;
      drive_normal();
      return;
    }
  }
  else
  {
    cv::Mat stop_line = frame_(cv::Rect(310, 350, 20, 40));
    cv::cvtColor(stop_line, stop_line, cv::COLOR_BGR2GRAY);
    cv::threshold(stop_line, stop_line, 200, 255, cv::THRESH_BINARY);
    std::cout << (int)cv::mean(stop_line)[0] << std::endl;
    if ((int)cv::mean(stop_line)[0] > 230) {
      std::cout << 1 << std::endl;
      drive_normal();
      return;
    }
  }

  int lpos, rpos, error, ma_mpos, left_mpos, right_mpos;
  float steering_angle;
  float max_cnt = 1.0f;
  int cnt = 0;

  while (static_cast<float>(cnt) < 1.0f) {
    int lpos, rpos;
    std::tie(lpos, rpos) = hough_transform_lane_detector_ptr_->getLanePosition(frame_);

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

  cnt = 0;

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

void LaneKeepingSystem::traffic_sign_recognition()
{
  int width = box_xmax - box_xmin;
  int height = box_ymax - box_ymin;
  cv::Mat traffic_light_upside = frame_(cv::Rect(0, 0, width, height/3));
  cv::Mat traffic_light_downside = frame_(cv::Rect(0, height * 2 / 3, width, height/3));
  
  cv::Mat hsv_upside, hsv_downside;
  std::vector<cv::Mat> hsv_upside_split, hsv_downside_split;
  
  cv::cvtColor(traffic_light_upside, hsv_upside, cv::COLOR_BGR2HSV);
  cv::cvtColor(traffic_light_downside, hsv_downside, cv::COLOR_BGR2HSV);

  cv::split(hsv_upside, hsv_upside_split);
  cv::split(hsv_downside, hsv_downside_split);

  float value_upside, value_downside;
  int w = traffic_light_upside.size().width;
  int h = traffic_light_upside.size().height;

  value_upside = cv::mean(hsv_upside_split[2](cv::Rect(w / 4, h / 4, w / 2, h / 2)))[0];
  value_downside = cv::mean(hsv_downside_split[2](cv::Rect(w / 4, h / 4, w / 2, h / 2)))[0];

  if(value_upside < 120 && value_downside < 120)
  {
    std::cout << "traffic light all off" << std::endl;
    std::cout << "v_upside : " << value_upside << "v_downside : " << value_downside << std::endl;
    
    return;
  }

  cv::Mat non_target;

  if (value_upside > value_downside)
  {
    non_target = hsv_downside_split[0];
  }
  else
  {
    non_target = hsv_upside_split[0];
  }


  cv::Mat red_mask, green_mask;
  cv::Scalar red_lower(140), red_upper(180);
  cv::Scalar green_lower(50), green_upper(90);

  cv::inRange(non_target, red_lower, red_upper, red_mask);
  cv::inRange(non_target, green_lower, green_upper, green_mask);

  int red_pixels = cv::countNonZero(red_mask);
  int green_pixels = cv::countNonZero(green_mask);


  // edit here --------------------------------------------------------------------
  if(red_pixels > green_pixels)
  {
    std::cout << "green light!" << std::endl;
    return;
  }
  else
  {
    std::cout << "red light!" << std::endl;
    return;
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

  past_steering_angle = motor_msg.angle;
  past_speed = motor_msg.speed;

  pub_.publish(motor_msg);
}
}  // namespace xycar
