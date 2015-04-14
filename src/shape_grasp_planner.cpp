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

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <simple_grasping/shape_grasp_planner.h>

using shape_msgs::SolidPrimitive;

namespace simple_grasping
{

moveit_msgs::GripperTranslation makeGripperTranslation(
  std::string frame,
  double min,
  double desired,
  double x_axis = 1.0,
  double y_axis = 0.0,
  double z_axis = 0.0)
{
  moveit_msgs::GripperTranslation translation;
  translation.direction.vector.x = x_axis;
  translation.direction.vector.y = y_axis;
  translation.direction.vector.z = z_axis;
  translation.direction.header.frame_id = frame;
  translation.min_distance = min;
  translation.desired_distance = desired;
  return translation;
}

Eigen::Quaterniond quaternionFromEuler(float yaw, float pitch, float roll)
{
  float sy = sin(yaw*0.5);
  float cy = cos(yaw*0.5);
  float sp = sin(pitch*0.5);
  float cp = cos(pitch*0.5);
  float sr = sin(roll*0.5);
  float cr = cos(roll*0.5);
  float w = cr*cp*cy + sr*sp*sy;
  float x = sr*cp*cy - cr*sp*sy;
  float y = cr*sp*cy + sr*cp*sy;
  float z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

ShapeGraspPlanner::ShapeGraspPlanner(ros::NodeHandle& nh)
{
  /*
   * Gripper model is based on having two fingers, and assumes
   * that the robot is using the moveit_simple_controller_manager
   * gripper interface, with "parallel" parameter set to true.
   */
  nh.param<std::string>("gripper/left_joint", left_joint_, "l_gripper_finger_joint");
  nh.param<std::string>("gripper/right_joint", right_joint_, "r_gripper_finger_joint");
  nh.param("gripper/max_opening", max_opening_, 0.110);
  nh.param("gripper/max_effort", max_effort_, 50.0);
  nh.param("gripper/finger_depth", finger_depth_, 0.02);
  nh.param("gripper/grasp_duration", grasp_duration_, 2.0);
  nh.param("gripper/gripper_tolerance", gripper_tolerance_, 0.02);

  /*
   * Approach is usually aligned with wrist_roll
   */
  nh.param<std::string>("gripper/approach/frame", approach_frame_, "wrist_roll_link");
  nh.param("gripper/approach/min", approach_min_translation_, 0.1);
  nh.param("gripper/approach/desired", approach_desired_translation_, 0.15);

  /*
   * Retreat is usually aligned with wrist_roll
   */
  nh.param<std::string>("gripper/retreat/frame", retreat_frame_, "wrist_roll_link");
  nh.param("gripper/retreat/min", retreat_min_translation_, 0.1);
  nh.param("gripper/retreat/desired", retreat_desired_translation_, 0.15);

  // Distance from tool point to planning frame
  nh.param("gripper/tool_to_planning_frame", tool_offset_, 0.165);
}

int ShapeGraspPlanner::createGrasp(const geometry_msgs::PoseStamped& pose,
                                   double gripper_opening,
                                   double gripper_pitch,
                                   double x_offset,
                                   double z_offset,
                                   double quality)
{
  moveit_msgs::Grasp grasp;
  grasp.grasp_pose = pose;

  // defaults
  grasp.pre_grasp_posture = makeGraspPosture(gripper_opening);
  grasp.grasp_posture = makeGraspPosture(0.0);
  grasp.pre_grasp_approach = makeGripperTranslation(approach_frame_,
                                                    approach_min_translation_,
                                                    approach_desired_translation_);
  grasp.post_grasp_retreat = makeGripperTranslation(retreat_frame_,
                                                    retreat_min_translation_,
                                                    retreat_desired_translation_,
                                                    -1.0);  // retreat is in negative x direction

  // initial pose
  Eigen::Affine3d p = Eigen::Translation3d(pose.pose.position.x,
                                           pose.pose.position.y,
                                           pose.pose.position.z) *
                        Eigen::Quaterniond(pose.pose.orientation.w,
                                           pose.pose.orientation.x,
                                           pose.pose.orientation.y,
                                           pose.pose.orientation.z);
  // translate by x_offset, 0, z_offset
  p = p * Eigen::Translation3d(x_offset, 0, z_offset);
  // rotate by 0, pitch, 0
  p = p * quaternionFromEuler(0.0, gripper_pitch, 0.0);
  // apply grasp point -> planning frame offset
  p = p * Eigen::Translation3d(-tool_offset_, 0, 0);

  grasp.grasp_pose.pose.position.x = p.translation().x();
  grasp.grasp_pose.pose.position.y = p.translation().y();
  grasp.grasp_pose.pose.position.z = p.translation().z();
  Eigen::Quaterniond q = (Eigen::Quaterniond)p.linear();
  grasp.grasp_pose.pose.orientation.x = q.x();
  grasp.grasp_pose.pose.orientation.y = q.y();
  grasp.grasp_pose.pose.orientation.z = q.z();
  grasp.grasp_pose.pose.orientation.w = q.w();

  grasp.grasp_quality = quality;

  grasps_.push_back(grasp);
  return 1;
}

// Create the grasps going in one direction around an object
// starts with gripper level, rotates it up
// this works for boxes and cylinders
int ShapeGraspPlanner::createGraspSeries(
  const geometry_msgs::PoseStamped& pose,
  double depth, double width, double height,
  bool use_vertical)
{
  int count = 0;

  // Gripper opening is limited
  if (width >= (max_opening_*0.9))
    return count;

  // Depth of grasp calculations
  double x = depth/2.0;
  double z = height/2.0;
  if (x > finger_depth_)
    x = finger_depth_ - x;
  if (z > finger_depth_)
    z = finger_depth_ - z;

  double open = std::min(width + gripper_tolerance_, max_opening_);

  // Grasp along top of box
  for (double step = 0.0; step < depth/2.0; step += 0.1)
  {
    if (use_vertical)
      count += createGrasp(pose, open, 1.57, step, -z, 1.0 - 0.1*step);  // vertical
    count += createGrasp(pose, open, 1.07, step, -z + 0.01, 0.7 - 0.1*step);  // slightly angled down
    if (step > 0.05)
    {
      if (use_vertical)
        count += createGrasp(pose, open, 1.57, -step, -z, 1.0 - 0.1*step);
      count += createGrasp(pose, open, 1.07, -step, -z + 0.01, 0.7 - 0.1*step);
    }
  }

  // Grasp horizontally along side of box
  for (double step = 0.0; step < height/2.0; step += 0.1)
  {
    count += createGrasp(pose, open, 0.0, x, step, 0.8 - 0.1*step);  // horizontal
    count += createGrasp(pose, open, 0.5, x-0.01, step, 0.6 - 0.1*step);  // slightly angled up
    if (step > 0.05)
    {
      count += createGrasp(pose, open, 0.0, x, -step, 0.8 - 0.1*step);
      count += createGrasp(pose, open, 0.5, x-0.01, -step, 0.6 - 0.1*step);
    }
  }

  // A grasp on the corner of the box
  count += createGrasp(pose, open, 1.57/2.0, x - 0.005, -z + 0.005, 0.25);

  return count;
}

int ShapeGraspPlanner::plan(const grasping_msgs::Object& object,
                            std::vector<moveit_msgs::Grasp>& grasps)
{
  ROS_INFO("shape grasp planning starting...");

  // Need a shape primitive
  if (object.primitives.size() == 0)
  {
    // Shape grasp planner can only plan for objects
    //  with SolidPrimitive bounding boxes
    return -1;
  }

  if (object.primitive_poses.size() != object.primitives.size())
  {
    // Invalid object
    return -1;
  }

  // Clear out internal vector
  grasps_.clear();

  // Setup Pose
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.header = object.header;
  grasp_pose.pose = object.primitive_poses[0];

  // Setup object orientation
  Eigen::Quaterniond q(object.primitive_poses[0].orientation.w,
                       object.primitive_poses[0].orientation.x,
                       object.primitive_poses[0].orientation.y,
                       object.primitive_poses[0].orientation.z);

  // Setup object dimensions
  double x, y, z;
  if (object.primitives[0].type == SolidPrimitive::BOX)
  {
    x = object.primitives[0].dimensions[SolidPrimitive::BOX_X];
    y = object.primitives[0].dimensions[SolidPrimitive::BOX_Y];
    z = object.primitives[0].dimensions[SolidPrimitive::BOX_Z];
  }
  else if (object.primitives[0].type == SolidPrimitive::CYLINDER)
  {
    x = y = 2.0 * object.primitives[0].dimensions[SolidPrimitive::CYLINDER_RADIUS];
    z = object.primitives[0].dimensions[SolidPrimitive::CYLINDER_HEIGHT];
  }

  // Generate grasps
  for (int i = 0; i < 4; ++i)
  {
    grasp_pose.pose.orientation.x = q.x();
    grasp_pose.pose.orientation.y = q.y();
    grasp_pose.pose.orientation.z = q.z();
    grasp_pose.pose.orientation.w = q.w();

    if (i < 2)
    {
      createGraspSeries(grasp_pose, x, y, z);
    }
    else
    {
      // Only two sets of unqiue vertical grasps
      createGraspSeries(grasp_pose, x, y, z, false);
    }

    // Next iteration, rotate 90 degrees about Z axis
    q = q * quaternionFromEuler(-1.57, 0.0, 0.0);
    std::swap(x, y);
  }

  ROS_INFO("shape grasp planning done.");

  grasps = grasps_;
  return grasps.size();  // num of grasps
}

trajectory_msgs::JointTrajectory
ShapeGraspPlanner::makeGraspPosture(double pose)
{
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names.push_back(left_joint_);
  trajectory.joint_names.push_back(right_joint_);
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(pose/2.0);
  point.positions.push_back(pose/2.0);
  point.effort.push_back(max_effort_);
  point.effort.push_back(max_effort_);
  point.time_from_start = ros::Duration(grasp_duration_);
  trajectory.points.push_back(point);
  return trajectory;
}

}  // namespace simple_grasping
