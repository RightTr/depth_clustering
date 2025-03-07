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

#include <ros/ros.h>

#include <string>

#include "ros_bridge/cloud_odom_ros_subscriber.h"

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "projections/ring_projection.h"
#include "projections/spherical_projection.h"
#include "utils/radians.h"
#include "visualization/cloud_saver.h"

#include "tclap/CmdLine.h"


using std::string;

using namespace depth_clustering;

int main(int argc, char* argv[]) {
  TCLAP::CmdLine cmd(
    "Subscribe to /velodyne_points topic and show clustering on the data.",
    ' ', "1.0");
  TCLAP::ValueArg<std::string> lidar_arg(
    "", "lidar",
    "Choose your lidar type(livox or velodyne)", true, "velodyne",
    "string");
  TCLAP::ValueArg<int> angle_arg(
    "", "angle",
    "Threshold angle. Below this value, the objects are separated", false, 10,
    "int");
  TCLAP::ValueArg<int> type_arg(
    "", "type", "For Velodyne, num of vertical beams in laser. One of: [16, 32, 64]. For Livox ,type of livox lidar. One of: [360, ...]",
    true, 0, "int");

  cmd.add(angle_arg);
  cmd.add(lidar_arg);
  cmd.add(type_arg);
  cmd.parse(argc, argv);
  std::unique_ptr<ProjectionParams> proj_params_ptr = nullptr;

  Radians angle_tollerance = Radians::FromDegrees(angle_arg.getValue());
  std::string mylidar = lidar_arg.getValue();;

  std::string topic_clouds;

  if(mylidar == "velodyne")
  {
    topic_clouds = "/velodyne_points";
    switch (type_arg.getValue()) 
    {
      case 16:
        proj_params_ptr = ProjectionParams::VLP_16();
        break;
      case 32:
        proj_params_ptr = ProjectionParams::HDL_32();
        break;
      case 64:
        proj_params_ptr = ProjectionParams::HDL_64();
        break;
  }
  }
  else if(mylidar == "livox")
  {
    topic_clouds = "/livox/lidar";
    switch (type_arg.getValue()) 
    {
      case 360:
        proj_params_ptr = ProjectionParams::MID_360();
        break;
      case 144:
        proj_params_ptr = ProjectionParams::HAP_144();
        break;
    }
  }

  if (!proj_params_ptr) {
    fprintf(stderr,
            "Params Error!");
    exit(1);
  }

  ros::init(argc, argv, "dynamics_processing");
  ros::NodeHandle nh;


  CloudOdomRosSubscriber subscriber(&nh, *proj_params_ptr, topic_clouds);

  int min_cluster_size = 20;
  int max_cluster_size = 100000;

  int smooth_window_size = 5;
  Radians ground_remove_angle = 5_deg;

  VectorCloudSaver cloud_saver("clusters", 10);
  CloudSaver original_saver("original_cloud");

  auto depth_ground_remover = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle, smooth_window_size);

  ImageBasedClusterer<LinearImageLabeler<>> clusterer(
      angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

  subscriber.AddClient(&depth_ground_remover);
  depth_ground_remover.AddClient(&clusterer);
  subscriber.AddClient(&original_saver);
  clusterer.AddClient(&cloud_saver);

  fprintf(stderr, "Running with angle tollerance: %f degrees\n",
          angle_tollerance.ToDegrees());

  subscriber.StartListeningToRos(mylidar);
  ros::spin();
  return 0;
}
