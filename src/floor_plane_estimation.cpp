/*********************************************************************
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *  Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */


#include <ros/ros.h>

#include "pointcloud_floor_filter/depth_to_pointcloud.h"
#include "pointcloud_floor_filter/floor_plane_estimation.h"

#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>

namespace floor_filtered_pointcloud
{

struct Point2D
{
  double x;
  double y;
};

struct Point3D
{
  double x;
  double y;
  double z;
};


FloorPlaneEstimation::FloorPlaneEstimation() :
    depth_converter_(new DepthToPointCloud())
{

}

FloorPlaneEstimation::~FloorPlaneEstimation()
{

}

void FloorPlaneEstimation::floorPlaneEstimation(const sensor_msgs::ImageConstPtr& depth_msg,
                                                sensor_msgs::CameraInfoConstPtr info_msg,
                                                geometry_msgs::TransformStampedPtr depth_to_odom_transform,
                                                double preselect_floor_distance,
                                                double filtered_floor_distance,
                                                sensor_msgs::PointCloud2& out_cloud,
                                                geometry_msgs::PointPtr& out_normal
                                                )
{

  if (info_msg)
  {
    depth_converter_->initialize(depth_msg, info_msg);

    tf::Transform depth_to_odom;
    tf::transformMsgToTF(depth_to_odom_transform->transform, depth_to_odom);

    const float* depth_ptr = reinterpret_cast<const float*>(&depth_msg->data[0]);

    std::size_t width = depth_msg->width;
    std::size_t height = depth_msg->height;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > > point_selection = boost::make_shared<pcl::PointCloud< pcl::PointXYZ > >();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > > cloud = boost::make_shared<pcl::PointCloud< pcl::PointXYZ > >();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > > obstacle = boost::make_shared<pcl::PointCloud< pcl::PointXYZ > >();

    point_selection->points.reserve(width*height);
    cloud->points.reserve(width*height);

    obstacle->points.reserve(width*height);

    for (size_t y=0; y<height; ++y)
      for (size_t x=0; x<width; ++x)
      {
        float depth = *(depth_ptr+
                        width*y+x);

        if (!isnan(depth))
        {
          Point2D p_in;
          p_in.x = x;
          p_in.y = y;

          Point3D p_out;
          depth_converter_->depthTo3DPoint<const Point2D, Point3D>(p_in, depth, p_out );

          tf::Point point_out( p_out.x, p_out.y, p_out.z);

          geometry_msgs::Point odom_point;
          tf::pointTFToMsg(depth_to_odom*point_out, odom_point);

          pcl::PointXYZ pcl_point(odom_point.x, odom_point.y, odom_point.z);

          if (fabs(odom_point.z)<preselect_floor_distance)
          {
            point_selection->points.push_back(pcl_point);
          }

          cloud->points.push_back(pcl_point);
        }
      }

    pcl::ModelCoefficients coeffs;
    pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();

    pcl::SACSegmentation < pcl::PointXYZ > seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(filtered_floor_distance);

    seg.setInputCloud(point_selection);
    seg.segment(*inliers, coeffs);

    for (size_t i=0; i<cloud->points.size(); ++i)
    {
      const pcl::PointXYZ& point = cloud->points[i];
      double dist = coeffs.values[0]*point.x +
                    coeffs.values[1]*point.y +
                    coeffs.values[2]*point.z +
                    coeffs.values[3];

      if (dist>filtered_floor_distance)
      {
        obstacle->points.push_back(point);
      }
    }

    pcl::toROSMsg (*obstacle, out_cloud); //convert the cloud

    out_normal = boost::make_shared<geometry_msgs::Point>();

    out_normal->x = coeffs.values[0];
    out_normal->y = coeffs.values[1];
    out_normal->z = coeffs.values[2];
  }

}

}

