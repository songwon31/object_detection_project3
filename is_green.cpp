void traffic_sign_recognition()
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