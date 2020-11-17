// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;


void imageCallback(boost::shared_ptr<sensor_msgs::Image> ros1_msg)
{
  if (pub->get_subscription_count() == 0)
    return;

  auto ros2_msg = std::make_unique<sensor_msgs::msg::Image>();

  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  ros2_msg->header.stamp = rclcpp::Time(ros1_msg->header.stamp.toNSec());
  
  ros2_msg->height = ros1_msg->height;
  ros2_msg->width = ros1_msg->width;
  ros2_msg->encoding = ros1_msg->encoding;
  ros2_msg->is_bigendian = ros1_msg->is_bigendian;
  ros2_msg->step = ros1_msg->step;

  std::vector<uint8_t> data(ros1_msg->data.begin(), ros1_msg->data.end());

  ros2_msg->data = data;

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("depth_1_to_2");
  pub = node->create_publisher<sensor_msgs::msg::Image>("/depth", rclcpp::SensorDataQoS());

  // ROS 1 node and subscriber
  ros::init(argc, argv, "depth_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/xtion/depth_registered/image_raw", 10, imageCallback);

  ros::spin();

  rclcpp::shutdown();

  return 0;
}
