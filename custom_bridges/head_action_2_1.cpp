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
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PointStamped.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <person_follower_interfaces/msg/bridge_action.hpp>


// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;
PointHeadClientPtr pointHeadClient;

void action_callback(const person_follower_interfaces::msg::BridgeAction::SharedPtr msg)
{
	control_msgs::PointHeadGoal goal;

	goal.target.header.frame_id = "/xtion_rgb_optical_frame";
	goal.target.header.stamp = ros::Time::now();

	goal.target.point.x = msg->point.x;
	goal.target.point.z = msg->point.z;
	goal.target.point.y = msg->point.y;

	goal.pointing_frame = "/xtion_rgb_optical_frame";
	goal.pointing_axis.x = 0;
	goal.pointing_axis.y = 0;
	goal.pointing_axis.z = 1;

	goal.max_velocity = msg->max_velocity;

	goal.min_duration = ros::Duration(msg->min_duration);
	
	pointHeadClient->sendGoal(goal);
}

void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );
}

int main(int argc, char * argv[])
{
	// ROS 1 node and client
	ros::init(argc, argv, "head_action_2_to_1");
	ros::NodeHandle n;
	createPointHeadClient(pointHeadClient);

	//ros::spin();

	//rclcpp::shutdown();

	// ROS 2 node and subscriber
    rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("head_action_2_to_1");
	auto sub = node->create_subscription<person_follower_interfaces::msg::BridgeAction>(
		"/head_move_action", 1, action_callback);

	rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}