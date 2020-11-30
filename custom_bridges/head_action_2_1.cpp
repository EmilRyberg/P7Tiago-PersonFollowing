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
#include <string>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/topic.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <person_follower_interfaces/msg/bridge_action.hpp>

static const std::string imageTopic      = "/xtion/rgb/image_raw";
static const std::string cameraInfoTopic = "/xtion/rgb/camera_info";

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;
PointHeadClientPtr pointHeadClient;
cv::Mat cameraIntrinsics;

void action_callback(const person_follower_interfaces::msg::BridgeAction::SharedPtr msg)
{
	geometry_msgs::PointStamped pointStamped;

	pointStamped.header.frame_id = "/xtion_rgb_optical_frame";
	pointStamped.header.stamp    = ros::Time::now();

	//compute normalized coordinates of the selected pixel
	double x = ( msg->x  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
	double y = ( msg->y  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
	double Z = 1.0; //define an arbitrary distance
	pointStamped.point.x = x * Z;
	pointStamped.point.y = y * Z;
	pointStamped.point.z = Z;  
	control_msgs::PointHeadGoal goal;

	goal.target.point.x = x * Z;
	goal.target.point.y = x * Z;
	goal.target.point.z = Z;

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
  actionClient.reset(new PointHeadClient("/head_controller/point_head_action") );
}

int main(int argc, char * argv[])
{
	// ROS 1 node and client
	ros::init(argc, argv, "head_action_2_to_1");
	ros::NodeHandle n;

	  // Get the camera intrinsic parameters from the appropriate ROS topic
	ROS_INFO("Waiting for camera intrinsics ... ");
	sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage
		<sensor_msgs::CameraInfo>(cameraInfoTopic, ros::Duration(10.0));
	if(msg.use_count() > 0)
	{
		cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
		cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
		cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
		cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
		cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
		cameraIntrinsics.at<double>(2, 2) = 1;
	}

	createPointHeadClient(pointHeadClient);

	// ROS 2 node and subscriber
    rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("head_action_2_to_1");
	auto sub = node->create_subscription<person_follower_interfaces::msg::BridgeAction>(
		"/head_move_action", 1, action_callback);

	rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}