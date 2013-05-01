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

#ifndef FLOOR_FILTER_LIB_H_
#define FLOOR_FILTER_LIB_H_

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include "boost/thread.hpp"

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>

#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/ros/conversions.h>

#include <dynamic_reconfigure/server.h>
#include <pointcloud_floor_filter/FloorFilteredPointCloudConfig.h>


namespace floor_filtered_pointcloud
{

class FloorPlaneEstimation;

class PointcloudFloorFilter
{
public:
  typedef pointcloud_floor_filter::FloorFilteredPointCloudConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  PointcloudFloorFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~PointcloudFloorFilter();

private:
  void cameraInfoCallback( const sensor_msgs::CameraInfoConstPtr& msg );
  void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg );

  void subscribe();
  void unsubscribe();

  void configCb(Config &config, uint32_t level);
  void connectCb();

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  boost::shared_ptr<FloorPlaneEstimation> floor_plane_estimation_;

  // ROS stuff
  boost::shared_ptr<image_transport::ImageTransport> depth_it_;
  boost::shared_ptr<image_transport::SubscriberFilter> depth_sub_;

  boost::shared_ptr< message_filters::Subscriber<sensor_msgs::CameraInfo> > camera_info_sub_;

  boost::mutex camera_info_mutex;
  sensor_msgs::CameraInfo::ConstPtr camera_info_msg;

  std::string odom_frame_;

  boost::shared_ptr<tf2::Buffer> tf_buffer_;
  boost::shared_ptr<tf2::TransformListener> tf_listener_;

  std::string depthmap_topic_;
  std::string depth_topic_transport_;

  double ransac_floor_distance_;
  double filtered_floor_distance_;

  ros::Publisher pointcloud_pub_;

  boost::mutex connect_mutex_;
};

}

#endif
