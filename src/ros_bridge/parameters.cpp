#include "parameters.h"

float centerz_threshold;
float dis2d_max_threshold;
float dis2d_min_threshold;
int min_cluster_size;
int max_cluster_size;
int smooth_window_size;
double ground_remove_angle_d;
bool is_mid360_custom_msg;
bool is_use_odometry;
bool is_mid360_tf;
float space_width;
float space_length;
float lidar2robot_x;
float lidar2robot_y;
string odom_topic;
string lidar_topic;


void ReadParameters(ros::NodeHandle &nh)
{
    nh.param<float>("depth_clustering/centerz_threshold", centerz_threshold, 2.0);
    nh.param<float>("depth_clustering/dis2d_max_threshold", dis2d_max_threshold, 3.0);
    nh.param<float>("depth_clustering/dis2d_min_threshold", dis2d_min_threshold, 2.0);
    nh.param<int>("depth_clustering/min_cluster_size", min_cluster_size, 50);
    nh.param<int>("depth_clustering/max_cluster_size", max_cluster_size, 1000);
    nh.param<int>("depth_clustering/smooth_window_size", smooth_window_size, 7);
    nh.param<double>("depth_clustering/ground_remove_angle_d", ground_remove_angle_d, 10);
    nh.param<bool>("lidar/is_mid360_custom_msg", is_mid360_custom_msg, true);
    nh.param<bool>("odometry/is_use_odometry", is_use_odometry, true);
    nh.param<bool>("odometry/is_mid360_tf", is_mid360_tf, true);
    nh.param<float>("odometry/space_width", centerz_threshold, 0);
    nh.param<float>("odometry/lidar2robot_x", lidar2robot_x, 0);
    nh.param<float>("odometry/lidar2robot_y", lidar2robot_y, 0);
    nh.param<string>("topic/odom_topic", odom_topic, "/aft_mapped_to_init");
    nh.param<string>("topic/lidar_topic", lidar_topic, "/livox/lidar");
}