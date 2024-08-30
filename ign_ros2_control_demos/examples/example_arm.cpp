// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

// Function to convert GoalUUID to a string
std::string uuid_to_string(const rclcpp_action::GoalUUID & uuid)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto & byte : uuid) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

void common_goal_response(
  rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::SharedPtr future)
{
  RCLCPP_DEBUG(
    node->get_logger(), "common_goal_response time: %f",
    rclcpp::Clock().now().seconds());
  auto goal_handle = future.get();
  if (!goal_handle) {
    common_goal_accepted = false;
    RCLCPP_ERROR(node->get_logger(), "Goal rejected by server.");
  } else {
    common_goal_accepted = true;
    RCLCPP_INFO(node->get_logger(), "Goal accepted by server.");
    RCLCPP_INFO(node->get_logger(), "Goal ID: %s", uuid_to_string(goal_handle->get_goal_id()).c_str());
  }
}

void common_result_response(
  const rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
  RCLCPP_INFO(node->get_logger(), "common_result_response time: %f", rclcpp::Clock().now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "SUCCEEDED result code.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted. Error code: %d", result.result->error_code);
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node->get_logger(), "Goal was canceled. Error code: %d", result.result->error_code);
      return;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code: %d", result.result->error_code);
      return;
  }
}

void common_feedback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  RCLCPP_INFO(node->get_logger(), "Feedback received:");
  std::cout << "feedback->desired.positions :";
  for (auto & x : feedback->desired.positions) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
  std::cout << "feedback->desired.velocities :";
  for (auto & x : feedback->desired.velocities) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("trajectory_test_node");

  std::cout << "node created" << std::endl;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "/joint_trajectory_controller/follow_joint_trajectory");

  bool response =
    action_client->wait_for_action_server(std::chrono::seconds(5));
  if (!response) {
    throw std::runtime_error("could not get action server");
  }
  std::cout << "Created action server" << std::endl;

  std::vector<std::string> joint_names = {"base_link_to_link_1"};

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

  // Initial position (0.0)
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
  point.positions.resize(joint_names.size());
  point.positions[0] = 0.0;

  // Final position (0.5)
  trajectory_msgs::msg::JointTrajectoryPoint point2;
  point2.time_from_start = rclcpp::Duration::from_seconds(1.0);  // 1 second later
  point2.positions.resize(joint_names.size());
  point2.positions[0] = 0.5;

  // Add both points to the trajectory
  points.push_back(point);
  points.push_back(point2);

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
  opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
  opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points = points;

  auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    action_client.reset();
    node.reset();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "send goal call ok :)");

  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server.");
    action_client.reset();
    node.reset();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Goal was accepted by server.");

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  }

  action_client.reset();
  node.reset();
  rclcpp::shutdown();

  return 0;
}
