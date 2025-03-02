#include "parameters.h"

float centerz_threshold;
float dis2d_threshold;
int min_cluster_size;
int max_cluster_size;
int smooth_window_size;
double ground_remove_angle_d;
bool is_mid360_custom_msg;

void ReadParameters(ros::NodeHandle &nh)
{
    nh.param<float>("depth_clustering/centerz_threshold", centerz_threshold, 2.0);
    nh.param<float>("depth_clustering/dis2d_threshold", dis2d_threshold, 3.0);
    nh.param<int>("depth_clustering/min_cluster_size", min_cluster_size, 50);
    nh.param<int>("depth_clustering/max_cluster_size", max_cluster_size, 1000);
    nh.param<int>("depth_clustering/smooth_window_size", smooth_window_size, 7);
    nh.param<double>("depth_clustering/ground_remove_angle_d", ground_remove_angle_d, 10);
    nh.param<bool>("lidar/is_mid360_custom_msg", is_mid360_custom_msg, true);
}