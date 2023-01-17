#include <ros/ros.h>

#include "xycar_msgs/xycar_motor.h"
#include "yolov3_trt_ros/BoundingBox.h"
#include "yolov3_trt_ros/BoundingBoxes.h"

// Global variable
constexpr int SLEEP_RATE =
  12;  // while loop will iterate "SLEEP_RATE" times in a second (ros::Rate)
constexpr int WIDTH = 640;  // WIDTH of image
int OBJECT_ID = 0;

void callback(const yolov3_trt_ros::BoundingBoxes& msg) {
  // TODO: Implement your code here
  //* EXAMPLE CODE
  for (auto& box : msg.bounding_boxes) {
    std::cout << "Class ID: " << box.id << std::endl;
    OBJECT_ID = box.id;
  }
}

void drive_normal(std::string& direction, float time, PID& pid);
std::pair<int, int> yourFunction();
void find_traffic_light();
void find_u_turn();
void find_cross_walk();
void drive_stop();
void yourPublishFunction(float steer_angle);
void yourControlFunction(int cte);

int main(int argc, char** argv) {
  ros::init(argc, argv, "trt_driver");
  ros::NodeHandle nh;
  ros::Publisher pub =
    nh.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 10);
  ros::Subscriber sub =
    nh.subscribe("/yolov3_trt_ros/detections", 10, callback);

  while (ros::ok()) {
    ros::spinOnce();

    // Start Drive
    // Left
    if (OBJECT_ID == 0) {
      drive_normal("left", 2.5);
    }
    // Right
    else if (OBJECT_ID == 1) {
      drive_normal("right", 2.5);
    }
    // Stop sign
    else if (OBJECT_ID == 2) {
      // make your own stop function
      drive_stop();
    }
    // Crosswalk sign
    else if (OBJECT_ID == 3) {
      // make your own cross walk function
      find_cross_walk();
    }
    // U-turn sign
    else if (OBJECT_ID == 4) {
      // make your own u-turn function
      find_u_turn();
    }
    // Traffic light
    else if (OBJECT_ID == 5) {
      // make your own traffic right function
      find_traffic_light();
    }
    // Reset OBJECT_ID
    OBJECT_ID = -1;
  }

  return 0;
}

void drive_normal(std::string direction, float time) {
  ros::spinOnce();
  // Set the rate variable
  ros::Rate rate(SLEEP_RATE);

  float max_cnt;
  int cnt = 0;

  if (time == 0) {  // time 0 means straight drive.
    max_cnt = 1;
  } else {
    // Set the maximun number of iteration in while loop.
    max_cnt = static_cast<float>(SLEEP_RATE) * time;
  }

  while (static_cast<float>(cnt) < max_cnt) {
    int lpos, rpos;
    std::tie(lpos, rpos) = yourFunction();

    // Left or Right
    if (direction == "left") {
      rpos = lpos + 470;
    } else if (direction == "right") {
      lpos = rpos - 470;
    }

    int mpos = (lpos + rpos) * 0.5;
    int cte = ma_pos - WIDTH * 0.5;
    float steer_angle = yourControlFunction(cte);

    steer_angle = std::max(-50.0f, std::min(steer_angle, 50.0f));
    yourPublishFunction(steer_angle);

    cnt++;
    rate.sleep();
  }
}
