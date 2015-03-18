/*
 * Copyright (c) 2014, Unbounded Robotics Inc.
 * Copyright (c) 2013, Michael E. Ferguson
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

#ifndef SIMPLE_GRASPING_OBJECT_SUPPORT_SEGMENTATION_H
#define SIMPLE_GRASPING_OBJECT_SUPPORT_SEGMENTATION_H

#include <vector>

#include <grasping_msgs/Object.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

namespace simple_grasping
{

/**
 *  @brief Class that segments a point cloud into objects and supporting surfaces.
 */  
class ObjectSupportSegmentation
{
public:

  /**
   *  @brief Constructor, loads pipeline using ROS parameters.
   *  @param nh Node handle to use for accessing parameters.
   */
  ObjectSupportSegmentation(ros::NodeHandle& nh);

  /**
   *  @brief Split a cloud into objects and supporting surfaces.
   *  @param cloud The point cloud to segment. This cloud should already be
   *         transformed into a coordinate frame where XY plane is horizontal.
   *  @param objects The vector to fill in with objects found.
   *  @param supports The vector to fill in with support surfaces found.
   *  @param object_cloud A colored cloud of objects found (if output_clouds).
   *  @param support_cloud A colored cloud of supports found (if output_clouds).
   */
  bool segment(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
               std::vector<grasping_msgs::Object>& objects,
               std::vector<grasping_msgs::Object>& supports,
               pcl::PointCloud<pcl::PointXYZRGB>& object_cloud,
               pcl::PointCloud<pcl::PointXYZRGB>& support_cloud,
               bool output_clouds);
  
private:
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_;
  pcl::SACSegmentation<pcl::PointXYZRGB> segment_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract_clusters_;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_;
};

}  // namespace simple_grasping

#endif  // SIMPLE_GRASPING_OBJECT_SUPPORT_SEGMENTATION_H
