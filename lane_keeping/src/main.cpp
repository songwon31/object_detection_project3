#include "lane_keeping_system/lane_keeping_system.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Lane Keeping System");
  xycar::LaneKeepingSystem lks;
  lks.run();

  return 0;
}