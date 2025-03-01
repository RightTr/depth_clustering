#include "parameters.h"

float centerz_threshold;
float dis2d_threshold;
int min_cluster_size;
int max_cluster_size;
int smooth_window_size;
double ground_remove_angle_d;

void ReadParameters(ros::NodeHandle &nh)
{
    nh.param<float>("centerz_threshold", centerz_threshold, 2.0);
    nh.param<float>("dis2d_threshold", dis2d_threshold, 3.0);
    nh.param<int>("min_cluster_size", min_cluster_size, 50);
    nh.param<int>("max_cluster_size", max_cluster_size, 1000);
    nh.param<int>("smooth_window_size", smooth_window_size, 7);
    nh.param<double>("ground_remove_angle_d", ground_remove_angle_d, 10);
    
}