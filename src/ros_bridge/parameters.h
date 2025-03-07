#ifndef SRC_PARAMETERS_H_
#define SRC_PARAMETERS_H_
#include <ros/ros.h>

using std::string;

extern float centerz_threshold;
extern float dis2d_max_threshold;
extern float dis2d_min_threshold;
extern int min_cluster_size;
extern int max_cluster_size;
extern int smooth_window_size;
extern double ground_remove_angle_d;
extern bool is_mid360_custom_msg;
extern bool is_use_odometry;
extern bool is_mid360_tf;
extern float space_width;
extern float space_length;
extern float lidar2robot_x;
extern float lidar2robot_y;
extern string odom_topic;
extern string lidar_topic;




void ReadParameters(ros::NodeHandle &nh);

#endif