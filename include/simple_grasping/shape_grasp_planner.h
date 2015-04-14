/*
 * Copyright 2015, Fetch Robotics Inc.
 * Copyright 2013-2014, Unbounded Robotics Inc.
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
 *     * Neither the name of the Unbounded Robotics, Inc. nor the names of its
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

#ifndef SIMPLE_GRASPING_SHAPE_GRASP_PLANNER_H
#define SIMPLE_GRASPING_SHAPE_GRASP_PLANNER_H

#include <ros/ros.h>
#include <grasping_msgs/GraspableObject.h>

namespace simple_grasping
{

/**
 *  @brief A simple grasp planner that uses the bounding box shape to
 *         generate viable grasps.
 */
class ShapeGraspPlanner
{
public:
  /**
   * @brief Constructor, loads grasp planner configuration from ROS params.
   * @param nh Nodehandle to use for accessing grasp planner parameters.
   */
  ShapeGraspPlanner(ros::NodeHandle& nh);

  virtual int plan(const grasping_msgs::Object& object,
                   std::vector<moveit_msgs::Grasp>& grasps);

private:
  /**
   *  @brief Generate a grasp, add it to internal grasps_
   *  @param pose The pose of the end effector tool point
   *  @param gripper_pitch The pitch of the gripper on approach
   *  @param x_offset The offset in the x direction (in).
   *  @param z_offset The offset in the z direction (up).
   *  @param quality The quality to ascribe to this grasp.
   *  @returns The number of grasps generated.
   */
  int createGrasp(const geometry_msgs::PoseStamped& pose,
                  double gripper_opening,
                  double gripper_pitch,
                  double x_offset,
                  double z_offset,
                  double quality);

  /**
   *  @brief Generate a series of grasps around the edge of a shape
   *  @param pose The pose of the end effector tool point.
   *  @param depth The depth of the shape.
   *  @param width The width of the shape.
   *  @param height The height of the shape.
   *  @param use_vertical Whether to include vertical poses. If coming
   *         from two sides, the second call probably should not generate
   *         vertical poses.
   *  @returns The number of grasps generated.
   */
  int createGraspSeries(const geometry_msgs::PoseStamped& pose,
                        double depth, double width, double height,
                        bool use_vertical = true);

  trajectory_msgs::JointTrajectory makeGraspPosture(double pose);

  // gripper model
  std::string left_joint_, right_joint_;
  double max_opening_;
  double max_effort_;
  double grasp_duration_;
  double tool_offset_;
  double finger_depth_;
  double gripper_tolerance_;

  // approach model
  std::string approach_frame_;
  double approach_min_translation_;
  double approach_desired_translation_;

  // retreat model
  std::string retreat_frame_;
  double retreat_min_translation_;
  double retreat_desired_translation_;

  // storing of internal grasps as we generate them
  std::vector<moveit_msgs::Grasp> grasps_;
};

}  // namespace simple_grasping

#endif  // SIMPLE_GRASPING_SHAPE_GRASP_PLANNER_H
