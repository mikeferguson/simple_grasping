/*
 * Copyright 2013-2014, Unbounded Robotics Inc.
 * Copyright 2011, Willow Garage, Inc. (hsv2rgb)
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

#ifndef SIMPLE_GRASPING_CLOUD_TOOLS_H
#define SIMPLE_GRASPING_CLOUD_TOOLS_H

#include <Eigen/Eigen>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <vector>

namespace simple_grasping
{

/**
 *  \brief Fill in RGB values from HSV values.
 *  \param h HSV hue input.
 *  \param s HSV saturation input.
 *  \param v HSV value input.
 *  \param r RGB red output.
 *  \param g RGB green output.
 *  \param b RGB blue output.
 */
void hsv2rgb(const float h, const float s, const float v, float& r, float& g, float& b);

/**
 *  \brief Update rgb component of an XYZRGB cloud to a new HSV color
 *  \param cloud The cloud to update.
 *  \param hue The hue to apply.
 */
void colorizeCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float hue);

/**
 *  \brief Get distance from point to plane
 *  \param point The point.
 *  \param plane Plane coefficients.
 */
template<typename PointT>
double distancePointToPlane(const PointT& point, const pcl::ModelCoefficients::Ptr plane)
{
  Eigen::Vector4f pp(point.x, point.y, point.z, 1);
  Eigen::Vector4f m(plane->values[0], plane->values[1], plane->values[2], plane->values[3]);
  return pp.dot(m);
}

/**
 *  \brief Get distance from point to plane
 *  \param point The point.
 *  \param plane Plane coefficients.
 */
double distancePointToPlane(const Eigen::Vector4f& point, const pcl::ModelCoefficients::Ptr plane);

}  // namespace simple_grasping

#endif  // SIMPLE_GRASPING_CLOUD_TOOLS_H
