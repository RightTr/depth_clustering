// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "ros_bridge/cloud_odom_ros_subscriber.h"
#include <eigen_conversions/eigen_msg.h>

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

#include "utils/pose.h"

extern bool is_mid360_custom_msg;
extern bool is_use_odometry;
extern bool is_mid360_tf;
extern float lidar2robot_x;
extern float lidar2robot_y;
float lidar2robot = sqrt(lidar2robot_x * lidar2robot_x
                          + lidar2robot_y * lidar2robot_y);
float lidar2robot_angle = atan2f(lidar2robot_y, lidar2robot_x);

namespace depth_clustering {

using ros::NodeHandle;
using message_filters::Subscriber;
using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using nav_msgs::Odometry;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;
using depth_clustering::CustomMsg;

using std::vector;
using std::string;
using std::map;

template <class T>
T BytesTo(const vector<uint8_t>& data, uint32_t start_idx) {
  const size_t kNumberOfBytes = sizeof(T);
  uint8_t byte_array[kNumberOfBytes];
  // forward bit order (it is a HACK. We do not account for bigendianes)
  for (size_t i = 0; i < kNumberOfBytes; ++i) {
    byte_array[i] = data[start_idx + i];
  }
  T result;
  std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
            reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
            reinterpret_cast<uint8_t*>(&result));
  return result;
}

void PrintMsgStats(const sensor_msgs::PointCloud2ConstPtr& msg) {
  fprintf(stderr, "<<<<<<<<<<<<<<< new cloud >>>>>>>>>>>>>>>\n");
  fprintf(stderr, "received msg   %d\n", msg->header.seq);
  fprintf(stderr, "height:        %d\n", msg->height);
  fprintf(stderr, "width:         %d\n", msg->width);
  fprintf(stderr, "num of fields: %lu\n", msg->fields.size());
  fprintf(stderr, "fields of each point:\n");
  for (auto const& pointField : msg->fields) {
    fprintf(stderr, "\tname:     %s\n", pointField.name.c_str());
    fprintf(stderr, "\toffset:   %d\n", pointField.offset);
    fprintf(stderr, "\tdatatype: %d\n", pointField.datatype);
    fprintf(stderr, "\tcount:    %d\n", pointField.count);
    fprintf(stderr, "\n");
  }
  fprintf(stderr, "is bigendian:  %s\n", msg->is_bigendian ? "true" : "false");
  fprintf(stderr, "point step:    %d\n", msg->point_step);
  fprintf(stderr, "row step:      %d\n", msg->row_step);
  fprintf(stderr, "data size:     %lu\n", msg->data.size() * sizeof(msg->data));
  fprintf(stderr, "is dense:      %s\n", msg->is_dense ? "true" : "false");
  fprintf(stderr, "=========================================\n");
}

CloudOdomRosSubscriber::CloudOdomRosSubscriber(NodeHandle* node_handle,
                                               const ProjectionParams& params,
                                               const string& topic_clouds,
                                               const string& topic_odom)
    : AbstractSender{SenderType::STREAMER}, _params{params} { //TODO: Input params
  _node_handle = node_handle;
  _topic_clouds = topic_clouds;
  _topic_odom = topic_odom;
  _msg_queue_size = 100;

  _subscriber_clouds_pcl2 = nullptr;
  _subscriber_clouds_custom = nullptr;
  _subscriber_odom = nullptr;
  _sync_pcl2 = nullptr;
  _sync_custom = nullptr;
}

void CloudOdomRosSubscriber::StartListeningToRos(const std::string mylidar) 
{
  _mylidar = mylidar;
  if (is_use_odometry) 
  {
    _subscriber_odom = new Subscriber<Odometry>(
      *_node_handle, _topic_odom, _msg_queue_size);
    if(mylidar == "velodyne")
    {
      _subscriber_clouds_pcl2 = new Subscriber<PointCloud2>(
        *_node_handle, _topic_clouds, _msg_queue_size);
      _sync_pcl2 = new Synchronizer<ApproximateTimePolicyPcl2>(
        ApproximateTimePolicyPcl2(100), *_subscriber_clouds_pcl2, *_subscriber_odom);
      _sync_pcl2->registerCallback(
        boost::bind(&CloudOdomRosSubscriber::CallbackVelodyneOdom, this, _1, _2));
    }
    else if(mylidar == "livox")
    { 
      if(is_mid360_custom_msg)
      {

        _subscriber_clouds_custom = new Subscriber<CustomMsgT>(
          *_node_handle, _topic_clouds, _msg_queue_size);
        _sync_custom = new Synchronizer<ApproximateTimePolicyCustom>(
          ApproximateTimePolicyCustom(100), *_subscriber_clouds_custom, *_subscriber_odom);
        _sync_custom->registerCallback(std::bind(static_cast<void (CloudOdomRosSubscriber::*)
          (const CustomMsgT::ConstPtr&, const OdometryT::ConstPtr&)>(&CloudOdomRosSubscriber::CallbackLivoxOdom), this, 
          std::placeholders::_1, std::placeholders::_2));
        return ; 
      }
      else
      {
        _subscriber_clouds_pcl2 = new Subscriber<PointCloud2>(
          *_node_handle, _topic_clouds, _msg_queue_size);
        _sync_pcl2 = new Synchronizer<ApproximateTimePolicyPcl2>(
          ApproximateTimePolicyPcl2(100), *_subscriber_clouds_pcl2, *_subscriber_odom);
        _sync_pcl2->registerCallback(std::bind(static_cast<void (CloudOdomRosSubscriber::*)
          (const PointCloudT::ConstPtr&, const OdometryT::ConstPtr&)>(&CloudOdomRosSubscriber::CallbackLivoxOdom), this, 
          std::placeholders::_1, std::placeholders::_2));
      }
    }
  } 
  else 
  {
    if(mylidar == "velodyne")
    {
      _subscriber_clouds_pcl2 = new Subscriber<PointCloud2>(
        *_node_handle, _topic_clouds, _msg_queue_size);
      _subscriber_clouds_pcl2->registerCallback(
          &CloudOdomRosSubscriber::CallbackVelodyne, this);
    }
    else if(mylidar == "livox")
    { 
      if(is_mid360_custom_msg)
      {
        _subscriber_clouds_custom = new Subscriber<CustomMsg>(
          *_node_handle, _topic_clouds, _msg_queue_size);
        _subscriber_clouds_custom->registerCallback(std::bind(static_cast<void (CloudOdomRosSubscriber::*)
          (const CustomMsgT::ConstPtr&)>(&CloudOdomRosSubscriber::CallbackLivox), this, std::placeholders::_1));
      }
      else
      {
        _subscriber_clouds_pcl2 = new Subscriber<PointCloud2>(
          *_node_handle, _topic_clouds, _msg_queue_size);
        _subscriber_clouds_pcl2->registerCallback(std::bind(static_cast<void (CloudOdomRosSubscriber::*)
          (const PointCloudT::ConstPtr&)>(&CloudOdomRosSubscriber::CallbackLivox), this, std::placeholders::_1));
      }
    }
  }
}

void CloudOdomRosSubscriber::CallbackVelodyneOdom(const PointCloud2::ConstPtr& msg_cloud,
                                      const Odometry::ConstPtr& msg_odom) 
{
  // PrintMsgStats(msg_cloud);
  Cloud::Ptr cloud_ptr = RosCloudToCloudRing(msg_cloud);
  cloud_ptr->SetSensorPose(RosOdomToPose(msg_odom));
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);
}

void CloudOdomRosSubscriber::CallbackLivoxOdom(const PointCloudT::ConstPtr& msg_cloud,
              const OdometryT::ConstPtr& msg_odom)
{
  Cloud::Ptr cloud_ptr = RosCloudToCloudIntensity(msg_cloud);
  cloud_ptr->SetSensorPose(RosOdomToPose(msg_odom));
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);
}

void CloudOdomRosSubscriber::CallbackLivoxOdom(const CustomMsgT::ConstPtr& msg_cloud,
              const OdometryT::ConstPtr& msg_odom)
{
  Cloud::Ptr cloud_ptr = RosCloudToCloudIntensity(msg_cloud);
  cloud_ptr->SetPose(RosOdomToPose(msg_odom));
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);
}

void CloudOdomRosSubscriber::CallbackVelodyne( //TODO:CallbackVelodyne
    const PointCloud2::ConstPtr& msg_cloud) {
  // PrintMsgStats(msg_cloud);
  Cloud::Ptr cloud_ptr = RosCloudToCloudRing(msg_cloud);
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);
}

Pose CloudOdomRosSubscriber::RosOdomToPose(const Odometry::ConstPtr& msg) {
  Pose pose;
  // we want float, so some casting is needed
  Eigen::Affine3d pose_double;
  Odometry::Ptr msg_clone(new Odometry(*msg));
  Sensor2Robot(msg_clone);
  tf::poseMsgToEigen(msg_clone->pose.pose, pose_double);
  pose = pose_double.cast<float>();
  return pose;
}

Cloud::Ptr CloudOdomRosSubscriber::RosCloudToCloudRing(
  const PointCloud2::ConstPtr& msg) {
  uint32_t x_offset = msg->fields[0].offset;
  uint32_t y_offset = msg->fields[1].offset;
  uint32_t z_offset = msg->fields[2].offset;
  uint32_t ring_offset = msg->fields[4].offset; //In MID360. This is Intensity.

  Cloud cloud;
  for (uint32_t point_start_byte = 0, counter = 0;
       point_start_byte < msg->data.size();
       point_start_byte += msg->point_step, ++counter) {
    RichPoint point;
    point.x() = BytesTo<float>(msg->data, point_start_byte + x_offset);
    point.y() = BytesTo<float>(msg->data, point_start_byte + y_offset);
    point.z() = BytesTo<float>(msg->data, point_start_byte + z_offset);
    point.ring() = BytesTo<uint16_t>(msg->data, point_start_byte + ring_offset);
    // point.z *= -1;  // hack
    cloud.push_back(point);
  }

  return make_shared<Cloud>(cloud);
}

Cloud::Ptr CloudOdomRosSubscriber::RosCloudToCloudIntensity(
  const PointCloud2::ConstPtr& msg)
{
  uint32_t x_offset = msg->fields[0].offset;
  uint32_t y_offset = msg->fields[1].offset;
  uint32_t z_offset = msg->fields[2].offset;
  uint32_t intensity_offset = msg->fields[4].offset;

  Cloud cloud;
  for (uint32_t point_start_byte = 0, counter = 0;
       point_start_byte < msg->data.size();
       point_start_byte += msg->point_step, ++counter) {
    RichPoint point;
    point.x() = BytesTo<float>(msg->data, point_start_byte + x_offset);
    point.y() = BytesTo<float>(msg->data, point_start_byte + y_offset);
    point.z() = BytesTo<float>(msg->data, point_start_byte + z_offset);
    point.intensity() = BytesTo<uint8_t>(msg->data, point_start_byte + intensity_offset);
    // point.z *= -1;  // hack
    cloud.push_back(point);
  }

  return make_shared<Cloud>(cloud);
}

Cloud::Ptr CloudOdomRosSubscriber::RosCloudToCloudIntensity(
  const CustomMsgT::ConstPtr& msg)
{
  Cloud cloud;
  for (auto point : msg->points) 
  {
    RichPoint richpoint;
    richpoint.x() = point.x;
    richpoint.y() = point.y;
    richpoint.z() = point.z;
    richpoint.intensity() = point.reflectivity;
    cloud.push_back(richpoint);
  }

  return make_shared<Cloud>(cloud);
}

void CloudOdomRosSubscriber::CallbackLivox(const sensor_msgs::PointCloud2::ConstPtr& msg_cloud_pcl2)
{
  Cloud::Ptr cloud_ptr = RosCloudToCloudIntensity(msg_cloud_pcl2);
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);
}

void CloudOdomRosSubscriber::CallbackLivox(const depth_clustering::CustomMsg::ConstPtr& msg_cloud_custom) //TODO:CallbackLivox
{
  Cloud::Ptr cloud_ptr = RosCloudToCloudIntensity(msg_cloud_custom);
  cloud_ptr->InitProjection(_params);
  ShareDataWithAllClients(*cloud_ptr);
}

void CloudOdomRosSubscriber::Sensor2Robot(OdometryT::Ptr& msg_odom)
{
  Eigen::Quaterniond q(msg_odom->pose.pose.orientation.w, msg_odom->pose.pose.orientation.x, 
                        msg_odom->pose.pose.orientation.y, msg_odom->pose.pose.orientation.z);
  float tf_x = 0.0;
  float tf_y = 0.0;
  float euler_z = atan2(2 * (q.y() * q.x() + q.w() * q.z()),
                   1 - 2 * (q.z() * q.z() + q.y() * q.y()));


  if(is_mid360_tf)
  {
    tf_x = float(msg_odom->pose.pose.position.x) + 0.011 - 0.025757 * cosf(atan2f(0.02329, 0.011) + euler_z);
    tf_y = float(msg_odom->pose.pose.position.y) + 0.02329 - 0.025757 * sinf(atan2f(0.02329, 0.011) + euler_z);
  }
  else
  {
    tf_x = float(msg_odom->pose.pose.position.x);
    tf_y = float(msg_odom->pose.pose.position.y);
  }

  tf_x = tf_x + lidar2robot_x - lidar2robot * cosf(lidar2robot_angle + euler_z);
  tf_y = tf_y + lidar2robot_y - lidar2robot * sinf(lidar2robot_angle + euler_z);

  msg_odom->pose.pose.position.x = double(tf_x);
  msg_odom->pose.pose.position.y = double(tf_y);
}



}  // namespace depth_clustering
