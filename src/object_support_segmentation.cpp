/*
 * Copyright 2015, Fetch Robotics Inc.
 * Copyright 2014, Unbounded Robotics Inc.
 * Copyright 2013, Michael E. Ferguson
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The names of the authors may not be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Michael Ferguson

#include <Eigen/Eigen>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <simple_grasping/cloud_tools.h>
#include <simple_grasping/shape_extraction.h>
#include <simple_grasping/object_support_segmentation.h>

namespace simple_grasping
{

ObjectSupportSegmentation::ObjectSupportSegmentation(
  ros::NodeHandle& nh)
{

  // cluster_tolerance: minimum separation distance of two objects
  double cluster_tolerance;
  nh.param<double>("cluster_tolerance", cluster_tolerance, 0.01);
  extract_clusters_.setClusterTolerance(cluster_tolerance);

  // cluster_min_size: minimum size of an object
  int cluster_min_size;
  nh.param<int>("cluster_min_size", cluster_min_size, 50);
  extract_clusters_.setMinClusterSize(cluster_min_size);

  // voxel grid the data before segmenting
  double leaf_size, llimit, ulimit;
  std::string field;
  nh.param<double>("voxel_leaf_size", leaf_size, 0.005);
  nh.param<double>("voxel_limit_min", llimit, 0.0);
  nh.param<double>("voxel_limit_max", ulimit, 1.8);
  nh.param<std::string>("voxel_field_name", field, "z");
  voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_.setFilterFieldName(field);
  voxel_grid_.setFilterLimits(llimit, ulimit);

  // segment objects
  segment_.setOptimizeCoefficients(true);
  segment_.setModelType(pcl::SACMODEL_PLANE);
  segment_.setMaxIterations(100);
  segment_.setDistanceThreshold(cluster_tolerance);
}

bool ObjectSupportSegmentation::segment(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
  std::vector<grasping_msgs::Object>& objects,
  std::vector<grasping_msgs::Object>& supports,
  pcl::PointCloud<pcl::PointXYZRGB>& object_cloud,
  pcl::PointCloud<pcl::PointXYZRGB>& support_cloud,
  bool output_clouds)
{
  ROS_INFO("object support segmentation starting...");

  // process the cloud with a voxel grid
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  voxel_grid_.setInputCloud(cloud);
  voxel_grid_.filter(*cloud_filtered);
  ROS_DEBUG("object_support_segmentation",
            "Filtered for transformed Z, now %d points.", static_cast<int>(cloud_filtered->points.size()));

  // remove support planes
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_horizontal_planes(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<pcl::ModelCoefficients::Ptr> plane_coefficients;  // coefs of all planes found
  int thresh = cloud_filtered->points.size()/8;
  while (cloud_filtered->points.size() > 500)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Segment the largest planar component from the remaining cloud
    segment_.setInputCloud(cloud_filtered);
    segment_.segment(*inliers, *coefficients);
    if (inliers->indices.size() < (size_t) thresh)  // TODO: make configurable? TODO make this based on "can we grasp object"
    {
      ROS_DEBUG("object_support_segmentation", "No more planes to remove.");
      break;
    }

    // Extract planar part for message
    pcl::PointCloud<pcl::PointXYZRGB> plane;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(plane);

    // Check plane is mostly horizontal
    Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    float angle = acos(Eigen::Vector3f::UnitZ().dot(normal));
    if (angle < 0.15)
    {
      ROS_DEBUG("object_support_segmentation",
                "Removing a plane with %d points.", static_cast<int>(inliers->indices.size()));

      // new support object, with cluster, bounding box, and plane
      grasping_msgs::Object object;
      pcl::toROSMsg(plane, object.point_cluster);
      // give the object a temporary name
      object.name = std::string("surface") + boost::lexical_cast<std::string>(supports.size());
      // add shape and pose
      shape_msgs::SolidPrimitive box;
      geometry_msgs::Pose pose;
      extractUnorientedBoundingBox(plane, box, pose);
      object.primitives.push_back(box);
      object.primitive_poses.push_back(pose);
      // add plane
      for (int i = 0; i < 4; ++i)
        object.surface.coef[i] = coefficients->values[i];
      // stamp and frame
      object.header.stamp = ros::Time::now();
      object.header.frame_id = cloud->header.frame_id;
      // add support surface to object list
      supports.push_back(object);

      if (output_clouds)
      {
        ROS_DEBUG("object_support_segmentation",
                  "Adding support cluster of size %d.", static_cast<int>(plane.points.size()));
        float hue = (360.0 / 8) * supports.size();
        colorizeCloud(plane, hue);
        support_cloud += plane;
      }

      // track plane for later use when determining objects
      plane_coefficients.push_back(coefficients);
    }
    else
    {
      // Add plane to temporary point cloud so we can recover points for object extraction below
      ROS_DEBUG("object_support_segmentation",
                "Plane is not horizontal");
      *non_horizontal_planes += plane;
    }

    // Extract the non-planar parts and proceed
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
  }
  ROS_DEBUG("object_support_segmentation",
            "Cloud now %d points.", static_cast<int>(cloud_filtered->points.size()));

  // Add the non-horizontal planes back in
  *cloud_filtered += *non_horizontal_planes;

  // Cluster
  std::vector<pcl::PointIndices> clusters;
  extract_clusters_.setInputCloud(cloud_filtered);
  extract_clusters_.extract(clusters);
  ROS_DEBUG("object_support_segmentation",
            "Extracted %d clusters.", static_cast<int>(clusters.size()));

  extract_indices_.setInputCloud(cloud_filtered);
  for (size_t i= 0; i < clusters.size(); i++)
  {
    // Extract object
    pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
    extract_indices_.setIndices(pcl::PointIndicesPtr(new pcl::PointIndices(clusters[i])));
    extract_indices_.filter(new_cloud);

    // Find centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(new_cloud, centroid);

    // Compare centroid to planes to find which plane we are supported by
    int support_plane_index = -1;
    double support_plane_distance = 1000.0;
    for (int p = 0; p < plane_coefficients.size(); ++p)
    {
      double distance = distancePointToPlane(centroid, plane_coefficients[p]);
      if (distance > 0.0 && distance < support_plane_distance)
      {
        support_plane_distance = distance;
        support_plane_index = p;
      }
    }

    if (support_plane_index == -1)
    {
      ROS_DEBUG("object_support_segmentation",
                "No support plane found for object");
      continue;
    }

    // new object, with cluster and bounding box
    grasping_msgs::Object object;
    // set name of supporting surface
    object.support_surface = std::string("surface") + boost::lexical_cast<std::string>(support_plane_index);
    // add shape, pose and transformed cluster
    shape_msgs::SolidPrimitive box;
    geometry_msgs::Pose pose;
    pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
    extractShape(new_cloud, plane_coefficients[support_plane_index], projected_cloud, box, pose);
    pcl::toROSMsg(projected_cloud, object.point_cluster);
    object.primitives.push_back(box);
    object.primitive_poses.push_back(pose);
    // add stamp and frame
    object.header.stamp = ros::Time::now();
    object.header.frame_id = cloud->header.frame_id;
    // add object to object list
    objects.push_back(object);

    if (output_clouds)
    {
      ROS_DEBUG("object_support_segmentation",
                "Adding an object cluster of size %d.", static_cast<int>(new_cloud.points.size()));
      float hue = (360.0 / clusters.size()) * i;
      colorizeCloud(new_cloud, hue);
      object_cloud += new_cloud;
    }
  }

  ROS_INFO("object support segmentation done processing.");
  return true;
}

}  // namespace simple_grasping
