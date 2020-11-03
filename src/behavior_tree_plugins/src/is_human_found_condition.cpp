// Copyright (c) 2018 Intel Corporation
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

#include <string>
#include <chrono>

#include "behavior_tree_plugins/is_human_found.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

IsHumanFoundCondition::IsHumanFoundCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_human_found_(false),
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  human_found_pub_ = this->create_publisher<std_msgs::msg::Bool>("human_found", 10);

  tracker_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "tracker/human_found",
    10,
    std::bind(&IsHumanFoundCondition::onDetectionRecieved, this, _1));

  RCLCPP_DEBUG(node_->get_logger(), "Initialized an IsHumanFoundCondition BT node");
}

IsHumanFoundCondition::~IsHumanFoundCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsHumanFoundCondition BT node");
}


void IsHumanFoundCondition::onOdomReceived(const typename std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "Human is found");
  is_human_found_(true);
  human_found_pub_->publish(msg)
}

BT::NodeStatus IsHumanFoundCondition::tick()
{
  // TODO(orduno) #383 Once check for is stuck and state calculations are moved to robot class
  //              this becomes
  // if (robot_state_.isStuck()) {

  if (is_human_found_) {
    logStuck("No human found");
    return BT::NodeStatus::SUCCESS;  // Successfully detected a stuck condition
  }

  logStuck("Human has been found");
  return BT::NodeStatus::FAILURE;  // Failed to detected a stuck condition
}

void IsHumanFoundCondition::logStuck(const std::string & msg) const
{
  static std::string prev_msg;

  if (msg == prev_msg) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), msg);
  prev_msg = msg;
}

bool IsHumanFoundCondition::isStuck()
{
  // TODO(orduno) #400 The robot getting stuck can result on different types of motion
  // depending on the state prior to getting stuck (sudden change in accel, not moving at all,
  // random oscillations, etc). For now, we only address the case where there is a sudden
  // harsh deceleration. A better approach to capture all situations would be to do a forward
  // simulation of the robot motion and compare it with the actual one.

  // Detect if robot bumped into something by checking for abnormal deceleration
  if (current_accel_ < brake_accel_limit_) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Current deceleration is beyond brake limit."
      " brake limit: %.2f, current accel: %.2f", brake_accel_limit_, current_accel_);

    return true;
  }

  return false;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsHumanFoundCondition>("IsHumanFound");
}