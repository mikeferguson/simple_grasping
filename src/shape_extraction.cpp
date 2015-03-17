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

#include <math.h>
#include <Eigen/Eigen>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <simple_grasping/shape_extraction.h>

namespace simple_grasping
{

bool extractShape(const pcl::PointCloud<pcl::PointXYZRGB>& input,
                  const pcl::ModelCoefficients::Ptr model,
                  pcl::PointCloud<pcl::PointXYZRGB>& output,
                  shape_msgs::SolidPrimitive& shape,
                  geometry_msgs::Pose& pose)
{
  // Used to decide between various shapes
  double min_volume = 1000.0;  // the minimum volume shape found thus far.
  Eigen::Matrix3f transformation;  // the transformation for the best-fit shape

  // Compute z height as maximum distance from planes
  double height = 0.0;
  for (size_t i = 0; i < input.size(); ++i)
  {
    Eigen::Vector4f pp(input[i].x, input[i].y, input[i].z, 1);
    Eigen::Vector4f m(model->values[0], model->values[1], model->values[2], model->values[3]);
    double distance_to_plane = fabs(pp.dot(m));
    if (distance_to_plane > height)
      height = distance_to_plane;
  }

  // Project object into 2d, using plane model coefficients
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr flat(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> projection;
  projection.setModelType(pcl::SACMODEL_PLANE);
  projection.setInputCloud(input.makeShared());  // stupid API
  projection.setModelCoefficients(model);
  projection.filter(*flat);

  // Rotate plane so that Z=0
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr flat_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Vector3f normal(model->values[0], model->values[1], model->values[2]);
  Eigen::Quaternionf qz; qz.setFromTwoVectors(normal, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f plane_rotation = qz.toRotationMatrix();
  Eigen::Matrix3f inv_plane_rotation = plane_rotation.inverse();

  for (size_t i = 0; i < flat->size(); ++i)
  {
    pcl::PointXYZRGB p;
    p.getVector3fMap() = plane_rotation * (*flat)[i].getVector3fMap();
    flat_projected->push_back(p);
  }

  // Find the convex hull
  pcl::PointCloud<pcl::PointXYZRGB> hull;
  pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
  convex_hull.setInputCloud(flat_projected);
  convex_hull.setDimension(2);
  convex_hull.reconstruct(hull);

  // Try fitting a rectangle
  shape_msgs::SolidPrimitive rect;  // the best-fit rectangle
  rect.type = rect.BOX;
  rect.dimensions.resize(3);
  for (size_t i = 0; i < hull.size() - 1; ++i)
  {
    // For each pair of hull points, determine the angle
    double rise = hull[i+1].y - hull[i].y;
    double run = hull[i+1].x - hull[i].x;
    // and normalize..
    {
      double l = sqrt((rise * rise) + (run * run));
      rise = rise/l;
      run = run/l;
    }

    // Build rotation matrix from change of basis
    Eigen::Matrix3f rotation;
    rotation(0, 0) = run;
    rotation(0, 1) = rise;
    rotation(0, 2) = 0.0;
    rotation(1, 0) = -rise;
    rotation(1, 1) = run;
    rotation(1, 2) = 0.0;
    rotation(2, 0) = 0.0;
    rotation(2, 1) = 0.0;
    rotation(2, 2) = 1.0;
    Eigen::Matrix3f inv_rotation = rotation.inverse();

    // Project hull to new coordinate system
    pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
    for (size_t j = 0; j < hull.size(); ++j)
    {
      pcl::PointXYZRGB p;
      p.getVector3fMap() = rotation * hull[j].getVector3fMap();
      projected_cloud.push_back(p);
    }

    // Compute min/max
    double x_min = 1000.0;
    double x_max = -1000.0;
    double y_min = 1000.0;
    double y_max = -1000.0;
    for (size_t j = 0; j < projected_cloud.size(); ++j)
    {
      if (projected_cloud[j].x < x_min)
        x_min = projected_cloud[j].x;
      if (projected_cloud[j].x > x_max)
        x_max = projected_cloud[j].x;

      if (projected_cloud[j].y < y_min)
        y_min = projected_cloud[j].y;
      if (projected_cloud[j].y > y_max)
        y_max = projected_cloud[j].y;
    }

    // Is this the best estimate?
    double area = (x_max - x_min) * (y_max - y_min);
    if (area*height < min_volume)
    {
      transformation = inv_plane_rotation * inv_rotation;

      rect.dimensions[0] = (x_max - x_min);
      rect.dimensions[1] = (y_max - y_min);
      rect.dimensions[2] = height;

      Eigen::Vector3f pose3f((x_max + x_min)/2.0, (y_max + y_min)/2.0,
                             projected_cloud[0].z + height/2.0);
      pose3f = transformation * pose3f;
      pose.position.x = pose3f(0);
      pose.position.y = pose3f(1);
      pose.position.z = pose3f(2);

      Eigen::Quaternionf q(transformation);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      min_volume = area * height;
      shape = rect;
    }
  }

  // Try fitting a cylinder
  shape_msgs::SolidPrimitive cylinder;  // the best-fit cylinder
  cylinder.type = cylinder.CYLINDER;
  cylinder.dimensions.resize(2);
  for (size_t i = 0; i < hull.size(); ++i)
  {
    for (size_t j = i + 1; j < hull.size(); ++j)
    {
      // For each pair of hull points determine the center point
      //  between them as a possible cylinder
      pcl::PointXYZRGB p;
      p.x = (hull[i].x + hull[j].x) / 2.0;
      p.y = (hull[i].y + hull[j].y) / 2.0;
      double radius = 0.0;
      // Find radius from this point
      for (size_t k = 0; k < hull.size(); ++k)
      {
        double dx = hull[k].x - p.x;
        double dy = hull[k].y - p.y;
        double r = sqrt((dx * dx) + (dy * dy));
        if (r > radius)
          radius = r;
      }
      // Is this cylinder the best match?
      double volume = M_PI * radius * radius * height;
      if (volume < min_volume)
      {
        transformation = inv_plane_rotation;

        cylinder.dimensions[0] = height;
        cylinder.dimensions[1] = radius;

        Eigen::Vector3f pose3f(p.x, p.y, hull[0].z + height/2.0);
        pose3f = transformation * pose3f;
        pose.position.x = pose3f(0);
        pose.position.y = pose3f(1);
        pose.position.z = pose3f(2);

        min_volume = volume;
        shape = cylinder;
      }
    }
  }

  // TODO: Try fitting a sphere?

  // Project input to new frame
  Eigen::Vector3f origin(pose.position.x, pose.position.y, pose.position.z);
  for (size_t j = 0; j < input.size(); ++j)
  {
    pcl::PointXYZRGB p;
    p.getVector3fMap() = transformation * (input[j].getVector3fMap() - origin);
    output.push_back(p);
  }
  return true;
}

bool extractShape(const pcl::PointCloud<pcl::PointXYZRGB>& input,
                  pcl::PointCloud<pcl::PointXYZRGB>& output,
                  shape_msgs::SolidPrimitive& shape,
                  geometry_msgs::Pose& pose)
{
  // Find lowest point, use as z height
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  coefficients->values.resize(4);
  coefficients->values[0] = 0.0;
  coefficients->values[1] = 0.0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 1000.0;  // z-height
  for (size_t i = 0; i < input.size(); ++i)
  {
    if (input[i].z < coefficients->values[3])
      coefficients->values[3] = input[i].z;
  }
  coefficients->values[3] = -coefficients->values[3];
  return extractShape(input, coefficients, output, shape, pose);
}

bool extractUnorientedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& input,
                                  shape_msgs::SolidPrimitive& shape,
                                  geometry_msgs::Pose& pose)
{
  double x_min = 1000.0;
  double x_max = -1000.0;
  double y_min = 1000.0;
  double y_max = -1000.0;
  double z_min = 1000.0;
  double z_max = -1000.0;

  for (size_t i = 0; i < input.size(); ++i)
  {
    if (input[i].x < x_min)
      x_min = input[i].x;
    if (input[i].x > x_max)
      x_max = input[i].x;

    if (input[i].y < y_min)
      y_min = input[i].y;
    if (input[i].y > y_max)
      y_max = input[i].y;

    if (input[i].z < z_min)
      z_min = input[i].z;
    if (input[i].z > z_max)
      z_max = input[i].z;
  }

  pose.position.x = (x_min + x_max)/2.0;
  pose.position.y = (y_min + y_max)/2.0;
  pose.position.z = (z_min + z_max)/2.0;

  shape.type = shape.BOX;
  shape.dimensions.push_back(x_max-x_min);
  shape.dimensions.push_back(y_max-y_min);
  shape.dimensions.push_back(z_max-z_min);

  return true;
}

}  // namespace simple_grasping
