/*
 * Copyright 2020, Michael Ferguson
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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "simple_grasping/object_support_segmentation.h"
#include "simple_grasping/shape_grasp_planner.h"
#include "grasping_msgs/action/find_graspable_objects.hpp"

#include "pcl_ros/transforms.hpp"
#include "pcl/common/io.h"
#include "pcl/filters/passthrough.h"

namespace simple_grasping
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("basic_grasping_perception");

using std::placeholders::_1;
using std::placeholders::_2;

/**
 *  \brief ROS wrapper for shape grasp planner + object support segmentation
 */
class BasicGraspingPerception : public rclcpp::Node
{
  using FindGraspableObjectsAction = grasping_msgs::action::FindGraspableObjects;
  using FindGraspableObjectsGoal = rclcpp_action::ServerGoalHandle<FindGraspableObjectsAction>;

public:
  explicit BasicGraspingPerception(const rclcpp::NodeOptions& options)
  : rclcpp::Node("basic_grasping_perception", options),
    debug_(false),
    find_objects_(false)
  {
    // Store clock
    clock_ = this->get_clock();

    // use_debug: enable/disable output of a cloud containing object points
    debug_ = this->declare_parameter<bool>("debug_topics", false);

    // frame_id: frame to transform cloud to (should be XY horizontal)
    world_frame_ = this->declare_parameter<std::string>("frame_id", "base_link");

    // Publish debugging views
    if (debug_)
    {
      rclcpp::QoS qos(1);
      qos.best_effort();
      object_cloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("object_cloud", qos);
      support_cloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("support_cloud", qos);
    }

    // Range filter for cloud
    range_filter_.setFilterFieldName("z");
    range_filter_.setFilterLimits(0, 2.5);

    // Setup TF2
    buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
    listener_.reset(new tf2_ros::TransformListener(*buffer_));

    // Subscribe to head camera cloud
    rclcpp::QoS points_qos(10);
    points_qos.best_effort();
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/head_camera/depth_registered/points",
      points_qos,
      std::bind(&BasicGraspingPerception::cloud_callback, this, _1));

    // Setup actionlib server
    server_ = rclcpp_action::create_server<FindGraspableObjectsAction>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "find_objects",
      std::bind(&BasicGraspingPerception::handle_goal, this, _1, _2),
      std::bind(&BasicGraspingPerception::handle_cancel, this, _1),
      std::bind(&BasicGraspingPerception::handle_accepted, this, _1)
    );

    RCLCPP_INFO(LOGGER, "basic_grasping_perception initialized");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Be lazy
    if (!find_objects_)
      return;

    // Convert to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*msg, *cloud);

    RCLCPP_DEBUG(LOGGER, "Cloud received with %d points.", static_cast<int>(cloud->points.size()));

    // Filter out noisy long-range points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    range_filter_.setInputCloud(cloud);
    range_filter_.filter(*cloud_filtered);
    RCLCPP_DEBUG(LOGGER, "Filtered for range, now %d points.",
                 static_cast<int>(cloud_filtered->points.size()));

    // Transform to grounded
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (!pcl_ros::transformPointCloud(world_frame_, *cloud_filtered, *cloud_transformed, *buffer_))
    {
      RCLCPP_ERROR(LOGGER, "Error transforming to frame %s", world_frame_.c_str());
      return;
    }

    // Run segmentation
    objects_.clear();
    supports_.clear();
    pcl::PointCloud<pcl::PointXYZRGB> object_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> support_cloud;
    if (debug_)
    {
      object_cloud.header.frame_id = cloud_transformed->header.frame_id;
      support_cloud.header.frame_id = cloud_transformed->header.frame_id;
    }
    segmentation_->segment(cloud_transformed,
                           objects_,
                           supports_,
                           object_cloud,
                           support_cloud,
                           debug_);

    if (debug_)
    {
      sensor_msgs::msg::PointCloud2 cloud_msg;

      pcl::toROSMsg(object_cloud, cloud_msg);
      object_cloud_pub_->publish(cloud_msg);

      pcl::toROSMsg(support_cloud, cloud_msg);
      support_cloud_pub_->publish(cloud_msg);
    }

    // Ok to continue processing
    find_objects_ = false;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FindGraspableObjectsAction::Goal> goal_handle)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<FindGraspableObjectsGoal> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<FindGraspableObjectsGoal> goal_handle)
  {
    if (!planner_ || !segmentation_)
    {
      // Create planner
      planner_.reset(new ShapeGraspPlanner(this->shared_from_this()));
      // Create perception
      segmentation_.reset(new ObjectSupportSegmentation(this->shared_from_this()));
    }

    // Break off a thread
    std::thread{std::bind(&BasicGraspingPerception::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<FindGraspableObjectsGoal> goal_handle)
  {
    auto result = std::make_shared<FindGraspableObjectsAction::Result>();

    // Get objects
    find_objects_ = true;
    rclcpp::Time t = clock_->now();
    while (find_objects_ == true)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (clock_->now() - t > rclcpp::Duration::from_seconds(3.0))
      {
        find_objects_ = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(LOGGER, "Failed to get camera data in allocated time.");
        return;
      }
    }

    const auto goal = goal_handle->get_goal();

    // Set object results
    for (size_t i = 0; i < objects_.size(); ++i)
    {
      grasping_msgs::msg::GraspableObject g;
      g.object = objects_[i];
      if (goal->plan_grasps)
      {
        // Plan grasps for object
        planner_->plan(objects_[i], g.grasps);
      }
      result->objects.push_back(g);
    }
    // Set support surfaces
    result->support_surfaces = supports_;

    goal_handle->succeed(result);
  }

  bool debug_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::string world_frame_;

  bool find_objects_;
  std::vector<grasping_msgs::msg::Object> objects_;
  std::vector<grasping_msgs::msg::Object> supports_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr support_cloud_pub_;

  std::shared_ptr<ShapeGraspPlanner> planner_;
  std::shared_ptr<ObjectSupportSegmentation> segmentation_;

  rclcpp_action::Server<FindGraspableObjectsAction>::SharedPtr server_;
  rclcpp::Clock::SharedPtr clock_;

  pcl::PassThrough<pcl::PointXYZRGB> range_filter_;
};

}  // namespace simple_grasping

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_grasping::BasicGraspingPerception)
