bool LaneKeepingSystem::is_green_light()
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

  cv::Mat h, v = planes[0], planes[2];

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(v, circles, cv::HOUGH_GRADIENT_ALT, 1.5, 10, 300, 0.9, 20, 50);

  int max_idx(-1);
  float max_val(0);
  cv::Rect max_rect;

  for (int i = 0; i < circles.size(); i++)
  {
    cv::Vec3i c = circles[i];

    cv::Rect rect(c[0] - c[2], c[1] - c[2], c[0] + c[2], c[1] + c[2]);

    float mean_v = cv::mean(v(rect))[0];
    if (mean_v > max_val)
    {
      max_idx = i;
      max_val = mean_v;
      max_rect = rect;
    }
  }

  h += 30;

  float mean_hue = cv::mean(h[max_rect]);
  if(mean_hue > 70 && mean_hue < 150)
  {
    return true;
  }
  else if(mean_hue > 0 && mean_hue < 60 )
  {
    return false
  }

  return false;
}