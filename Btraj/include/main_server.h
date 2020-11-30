#ifndef MAIN_SERVER_H
#define MAIN_SERVER_H

#include <ros/ros.h>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <algorithm>

const double goalDisPower = 100;
const double reGoalDisPOwer = 25;

double init_x,init_y;
bool isInit=false,havePath=false,haveTraj=false;

ros::Subscriber car_pose_sub_,traj_sub_,path_sub_,fmm_path_sub_,error_sub_;
ros::Publisher car_odo_pub_,traj_pub_,goal_pub_;
tf2_ros::TransformBroadcaster *br;
geometry_msgs::TransformStamped trans_m_w,trans_w_b,trans_b_s;

nav_msgs::Odometry car_odo;
std::vector<geometry_msgs::PoseStamped> global_path_latest;
std::vector<geometry_msgs::PoseStamped> fmm_path_latest;
bool is_global_path_valid = false,is_fmm_path_valid = false;
size_t lastGoal,nowGoal;

bool OverRange(geometry_msgs::Point a, geometry_msgs::Point b, double rangepower);
void car_pose_update(const geometry_msgs::PoseStampedPtr pose);
void marker_to_path(const visualization_msgs::MarkerPtr traj);
void global_path_update(const nav_msgs::PathPtr gloabal_path);

#endif // MAIN_SERVER_H
