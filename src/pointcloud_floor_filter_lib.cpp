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

#include "pointcloud_floor_filter/pointcloud_floor_filter_lib.h"

#include "pointcloud_floor_filter/floor_plane_estimation.h"

#include <image_transport/camera_common.h>

#include <message_filters/subscriber.h>

namespace floor_filtered_pointcloud
{

PointcloudFloorFilter::PointcloudFloorFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh),
    floor_plane_estimation_(new FloorPlaneEstimation()),
    tf_buffer_(new tf2::Buffer()),
    tf_listener_(  new tf2::TransformListener(*tf_buffer_)),
    ransac_floor_distance_(0.15),
    filtered_floor_distance_(0.10)
{

  // read depth map topic from param server
  pnh_.param<std::string>("depth_image", depthmap_topic_, "/head_camera/depth_registered/image");

  // read depth transport from param server
  pnh_.param<std::string>("depth_topic_transport", depth_topic_transport_, "compressedDepth");

  // read odom frame from param server
  pnh_.param<std::string>("base_footprint_frame", odom_frame_, "base_footprint");

  if (odom_frame_[0] == '/')
    odom_frame_ = odom_frame_.substr(1);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointcloudFloorFilter::connectCb, this);

  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("floor_filtered_pointcloud", 0, connect_cb, connect_cb);

  // Initialize dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(pnh_));
  reconfigure_server_->setCallback(boost::bind(&PointcloudFloorFilter::configCb, this, _1, _2));

}

PointcloudFloorFilter::~PointcloudFloorFilter()
{

}

void PointcloudFloorFilter::configCb(Config &config, uint32_t level)
{
  ransac_floor_distance_ = config.ransac_threshold;
  filtered_floor_distance_ = config.filter_threshold;
}

void PointcloudFloorFilter::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

   if (!pointcloud_pub_.getNumSubscribers())
   {
     unsubscribe();
   }
   else
   {
     subscribe();
   }
}


void PointcloudFloorFilter::subscribe()
{
  unsubscribe();

  // subscribe to image topics
  depth_sub_->subscribe(*depth_it_,
                        depthmap_topic_,
                        1,
                        image_transport::TransportHints(depth_topic_transport_));

  depth_sub_->registerCallback(boost::bind(&PointcloudFloorFilter::depthImageCallback, this, _1));

  // subscribe to CameraInfo  topic
  camera_info_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());

  std::string info_topic = image_transport::getCameraInfoTopic(depthmap_topic_);
  camera_info_sub_->subscribe(nh_, info_topic, 1);
  camera_info_sub_->registerCallback(boost::bind(&PointcloudFloorFilter::cameraInfoCallback, this, _1));

}

void PointcloudFloorFilter::unsubscribe()
{
  depth_it_.reset(new image_transport::ImageTransport(nh_));
  depth_sub_.reset(new image_transport::SubscriberFilter());

  camera_info_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());
}


void PointcloudFloorFilter::cameraInfoCallback( const sensor_msgs::CameraInfoConstPtr& msg )
{
  boost::mutex::scoped_lock l(camera_info_mutex);
  camera_info_msg = msg;
}

void PointcloudFloorFilter::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg )
{
  geometry_msgs::TransformStampedPtr depth_to_odom_transform(new geometry_msgs::TransformStamped);

  try
  {

    std::string source_frame = depth_msg->header.frame_id;
    if (source_frame[0] == '/')
      source_frame = source_frame.substr(1);

    try
    {
      // lookup transformation for tf_pair
      *depth_to_odom_transform = tf_buffer_->lookupTransform(odom_frame_, source_frame, depth_msg->header.stamp,
                                                             ros::Duration(3.0));

      sensor_msgs::PointCloud2 floor_filtered_pointcloud;
      geometry_msgs::PointPtr floor_normal(new geometry_msgs::Point);

      floor_plane_estimation_->floorPlaneEstimation(depth_msg,
                                                    camera_info_msg,
                                                    depth_to_odom_transform,
                                                    ransac_floor_distance_,
                                                    filtered_floor_distance_,
                                                    floor_filtered_pointcloud,
                                                    floor_normal);

      floor_filtered_pointcloud.header.stamp = ros::Time::now();
      floor_filtered_pointcloud.header.frame_id = odom_frame_;

      pointcloud_pub_.publish(floor_filtered_pointcloud);
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR("Transform failure: %s", ex.what());
      return;
    }
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

}
