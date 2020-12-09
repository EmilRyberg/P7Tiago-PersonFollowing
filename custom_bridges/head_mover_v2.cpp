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

// include ROS 1
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

ros::Publisher pub;

void callback(const geometry_msgs::msg::Vector3::SharedPtr ros2_msg)
{
	if (pub.getNumSubscribers() == 0)
		return;

	trajectory_msgs::JointTrajectory ros1_msg;
	ros1_msg.joint_names.resize(2);
	ros1_msg.joint_names[0]="head_1_joint";
	ros1_msg.joint_names[1]="head_2_joint";

	ros1_msg.points.resize(1);
	ros1_msg.points[0].positions.resize(2);
	ros1_msg.points[0].positions[0] = ros2_msg->x;
	ros1_msg.points[0].positions[1] = ros2_msg->y;

	ros1_msg.points[0].time_from_start = ros::Duration(0.25);

	//ros1_msg.linear.x = ros2_msg->linear.x;

	pub.publish(ros1_msg);
}

int main(int argc, char *argv[])
{
	// ROS 1 node and publisher
	ros::init(argc, argv, "head_mover_v2");
	ros::NodeHandle n;
	pub = n.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1);

	// ROS 2 node and subscriber
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("head_mover_v2");
	auto sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"/head_move", 1, callback);

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
