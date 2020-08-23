/*
 * Copyright 2020, Michael Ferguson
 * Copyright 2015, Fetch Robotics Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fetch Robotics, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Michael Ferguson

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simple_grasping/shape_grasp_planner.h"
#include "grasping_msgs/action/grasp_planning.hpp"

namespace simple_grasping
{

using std::placeholders::_1;
using std::placeholders::_2;

class GraspPlannerNode : public rclcpp::Node
{
  using GraspPlanningAction = grasping_msgs::action::GraspPlanning;
  using GraspPlanningGoal = rclcpp_action::ServerGoalHandle<GraspPlanningAction>;

public:
  explicit GraspPlannerNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("grasp_planner", options)
  {
    // Action for grasp planning
    server_ = rclcpp_action::create_server<GraspPlanningAction>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "plan",
      std::bind(&GraspPlannerNode::handle_goal, this, _1, _2),
      std::bind(&GraspPlannerNode::handle_cancel, this, _1),
      std::bind(&GraspPlannerNode::handle_accepted, this, _1)
    );
  }

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GraspPlanningAction::Goal> goal_handle)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GraspPlanningGoal> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GraspPlanningGoal> goal_handle)
  {
    auto result = std::make_shared<GraspPlanningAction::Result>();

    if (!planner_)
    {
      planner_.reset(new simple_grasping::ShapeGraspPlanner(this->shared_from_this()));
    }

    const auto goal = goal_handle->get_goal();
    planner_->plan(goal->object, result->grasps);
    goal_handle->succeed(result);
  }

  std::shared_ptr<simple_grasping::ShapeGraspPlanner> planner_;
  rclcpp_action::Server<GraspPlanningAction>::SharedPtr server_;
};

}  // namespace simple_grasping

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_grasping::GraspPlannerNode)
