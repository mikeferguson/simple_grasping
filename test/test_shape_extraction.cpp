/*
 * Copyright 2014, Unbounded Robotics Inc.
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

#include <gtest/gtest.h>
#include <simple_grasping/shape_extraction.h>

TEST(test_shape_extraction, simple_cube)
{
  // simple cube 4 corners of cube, plus one extra point
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointXYZRGB p;
  p.x = 0.2;
  p.y = 0.1;
  p.z = 0.1;
  cloud.push_back(p);
  p.x = 0.25;
  p.y = 0.1;
  cloud.push_back(p);
  p.x = 0.25;
  p.y = 0.15;
  cloud.push_back(p);
  p.x = 0.2;
  p.y = 0.15;
  cloud.push_back(p);
  p.x = 0.225;
  p.y = 0.14;
  cloud.push_back(p);

  pcl::PointCloud<pcl::PointXYZRGB> output;

  // create a plane at z = 0.05
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  plane->values.resize(4);
  plane->values[0] = 0.0;
  plane->values[1] = 0.0;
  plane->values[2] = 1.0;
  plane->values[3] = -0.05;

  shape_msgs::SolidPrimitive shape;
  geometry_msgs::Pose pose;

  bool ret = simple_grasping::extractShape(cloud, plane, output, shape, pose);
  EXPECT_TRUE(ret);

  EXPECT_EQ(shape.BOX, shape.type);
  ASSERT_EQ(3, shape.dimensions.size());
  EXPECT_FLOAT_EQ(0.05, shape.dimensions[0]);
  EXPECT_FLOAT_EQ(0.05, shape.dimensions[1]);
  EXPECT_FLOAT_EQ(0.05, shape.dimensions[2]);

  EXPECT_FLOAT_EQ(0.225, pose.position.x);
  EXPECT_FLOAT_EQ(0.125, pose.position.y);
  EXPECT_FLOAT_EQ(0.075, pose.position.z);

  EXPECT_FLOAT_EQ(0.0, pose.orientation.x);
  EXPECT_FLOAT_EQ(0.0, pose.orientation.y);
  EXPECT_FLOAT_EQ(0.0, pose.orientation.z);
  EXPECT_FLOAT_EQ(1.0, pose.orientation.w);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
