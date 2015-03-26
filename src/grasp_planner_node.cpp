/*
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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <simple_grasping/shape_grasp_planner.h>
#include <grasping_msgs/GraspPlanningAction.h>

class GraspPlannerNode
{
  typedef actionlib::SimpleActionServer<grasping_msgs::GraspPlanningAction> server_t;

public:
  GraspPlannerNode(ros::NodeHandle n)
  {
    // Planner
    planner_.reset(new simple_grasping::ShapeGraspPlanner(n));

    // Action for grasp planning
    server_.reset(new server_t(n, "plan",
                               boost::bind(&GraspPlannerNode::executeCallback, this, _1),
                               false));
    server_->start();
  }

private:
  void executeCallback(const grasping_msgs::GraspPlanningGoalConstPtr& goal)
  {
    grasping_msgs::GraspPlanningResult result;
    planner_->plan(goal->object, result.grasps);
    server_->setSucceeded(result);
  }

  boost::shared_ptr<simple_grasping::ShapeGraspPlanner> planner_;
  boost::shared_ptr<server_t> server_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grasp_planner");
  ros::NodeHandle nh("~");
  GraspPlannerNode planning(nh);
  ros::spin();
  return 0;
}
