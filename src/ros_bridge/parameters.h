#ifndef SRC_PARAMETERS_H_
#define SRC_PARAMETERS_H_
#include <ros/ros.h>


extern float centerz_threshold;
extern float dis2d_threshold;
extern int min_cluster_size;
extern int max_cluster_size;
extern int smooth_window_size;
extern double ground_remove_angle_d;
extern bool is_mid360_custom_msg;

void ReadParameters(ros::NodeHandle &nh);

#endif