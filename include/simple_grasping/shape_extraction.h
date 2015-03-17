/*
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

#ifndef SIMPLE_GRASPING_SHAPE_EXTRACTION_H
#define SIMPLE_GRASPING_SHAPE_EXTRACTION_H

#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace simple_grasping
{

/**
 *  \brief Find the smallest shape primitive we can fit around this object.
 *  \param input Point Cloud of the object.
 *  \param output Point Cloud transformed to the pose frame of the shape primitive fit.
 *  \param shape Returned smallest shape primitive fit.
 *  \param pose The pose of the shape primitive fit.
 *  \returns True if a shape was extracted, false if we have a failure.
 */
bool extractShape(const pcl::PointCloud<pcl::PointXYZRGB>& input,
                  pcl::PointCloud<pcl::PointXYZRGB>& output,
                  shape_msgs::SolidPrimitive& shape,
                  geometry_msgs::Pose& pose);

/**
 *  \brief Find the smallest shape primitive we can fit around this object, given
 *         the plane parameters.
 *  \param input Point Cloud of the object.
 *  \param model Model coefficients for the plane.
 *  \param output Point Cloud transformed to the pose frame of the shape primitive fit.
 *  \param shape Returned smallest shape primitive fit.
 *  \param pose The pose of the shape primitive fit.
 *  \returns True if a shape was extracted, false if we have a failure.
 */
bool extractShape(const pcl::PointCloud<pcl::PointXYZRGB>& input,
                  const pcl::ModelCoefficients::Ptr model,
                  pcl::PointCloud<pcl::PointXYZRGB>& output,
                  shape_msgs::SolidPrimitive& shape,
                  geometry_msgs::Pose& pose);

/**
 *  \brief Find a bounding box around a cloud. This method does not attempt to
 *         find best orientation and so the bounding box will be oriented with
 *         the frame of the cloud.
 *  \param input Point Cloud of the object.
 *  \param shape The box fit to this cloud.
 *  \param pose The pose of the shape primitive extracted.
 */
bool extractUnorientedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& input,
                                  shape_msgs::SolidPrimitive& shape,
                                  geometry_msgs::Pose& pose);

}  // namespace simple_grasping

#endif  // SIMPLE_GRASPING_SHAPE_EXTRACTION_H
