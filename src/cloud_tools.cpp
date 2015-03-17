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

#include <simple_grasping/cloud_tools.h>

namespace simple_grasping
{

// http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
// for points on a dark background you want somewhat lightened
// colors generally... back off the saturation (s)      
void hsv2rgb(const float h, const float s, const float v, float& r, float& g, float& b)
{
  float c = v * s;
  float hprime = h/60.0;
  float x = c * (1.0 - fabs(fmodf(hprime, 2.0f) - 1));

  r = g = b = 0;

  if (hprime < 1) {
    r = c; g = x;
  } else if (hprime < 2) {
    r = x; g = c;
  } else if (hprime < 3) {
    g = c; b = x;
  } else if (hprime < 4) {
    g = x; b = c;
  } else if (hprime < 5) {
    r = x; b = c;
  } else if (hprime < 6) {
    r = c; b = x;
  }

  float m = v - c;
  r += m; g+=m; b+=m;
}

void colorizeCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float hue)
{
  std::vector<pcl::PCLPointField> fields;
  pcl::getFields(cloud, fields);
  size_t rgb_field_index;
  for (rgb_field_index = 0; rgb_field_index < fields.size(); ++rgb_field_index)
  {
    if (fields[rgb_field_index].name == "rgb" ||
        fields[rgb_field_index].name == "rgba")
      break;
  }

  float r, g, b;
  hsv2rgb(hue, 0.8 /*saturation*/, 1.0 /*value*/, r, g, b);

  for (size_t j = 0; j < cloud.points.size(); ++j)
  {
    pcl::PointXYZRGB &p = cloud.points[j];
    unsigned char* pt_rgb = (unsigned char*) &p;
    pt_rgb += fields[rgb_field_index].offset;
    (*pt_rgb) = (unsigned char) (r * 255);
    (*(pt_rgb+1)) = (unsigned char) (g * 255);
    (*(pt_rgb+2)) = (unsigned char) (b * 255);
  }
}

double distancePointToPlane(const Eigen::Vector4f& point, const pcl::ModelCoefficients::Ptr plane)
{
  Eigen::Vector4f pp(point);
  pp[3] = 1.0;
  Eigen::Vector4f m(plane->values[0], plane->values[1], plane->values[2], plane->values[3]);
  return pp.dot(m);
}

}  // namespace simple_grasping
