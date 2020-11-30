#include "ros/ros.h"
#include "pathtracker.h"
#include "nav_msgs/Path.h"
#include <tf/transform_datatypes.h>
#include "move_base/move_base.h"
#include "nav_msgs/Odometry.h"
#include <algorithm>

PathTracker track;

void global_path_update(const nav_msgs::PathPtr &path_in)
{
  track.resetTracker();
  track.path_latest = path_in->poses;
  std::reverse(track.path_latest.begin(),track.path_latest.end());
  track.isValid=true;
}

void local_path_update(const nav_msgs::PathPtr &path_in)
{
  track.resetTracker();
  track.path_latest = path_in->poses;
  track.isValid=true;
}

void pose_update(const geometry_msgs::PoseStampedPtr &pose_in)
{
  track.car_pose = *pose_in;
  track.car_x = pose_in->pose.position.x;
  track.car_y = pose_in->pose.position.y;
  track.car_a = tf::getYaw(pose_in->pose.orientation);
}
void odo_update(const nav_msgs::OdometryPtr &odo_in)
{

  track.car_x = odo_in->pose.pose.position.x;
  track.car_y = odo_in->pose.pose.position.y;
  track.car_a = tf::getYaw(odo_in->pose.pose.orientation);
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"track_node");
  ros::NodeHandle nh;
  ros::Rate r(5);

  std::string _track_mode;
  nh.getParam("/track_node/track_mode", _track_mode);


  ros::Subscriber path_sub;
  ros::Subscriber odo_sub;
  ros::Subscriber pose_sub;


  if(_track_mode == "local")
  {
    path_sub = nh.subscribe("/local_path",1000,local_path_update);
    odo_sub  = nh.subscribe("/car_odo",1000,odo_update);
  }
  else
  {
    path_sub = nh.subscribe("/global_path",1000,global_path_update);
    pose_sub = nh.subscribe("/car_pose",1000,pose_update);
  }


  ros::Publisher cmd_pub = nh.advertise<move_base::move_base>("/move_base",10);
  move_base::move_base cmd;
  cmd.velocity = 300;
  cmd.header.frame_id = "base_link";
  cmd.isEmerge=false;

  while(ros::ok())
  {
    if(track.isValid == true)
    {
      track.calPrepoint();
      cmd.angle = track.calSteer();
      cmd.velocity = 300;
      cmd.isEmerge=false;

      cmd_pub.publish(cmd);
    }
    else
    {
      cmd.velocity = 0;
      cmd.isEmerge=true;

      cmd_pub.publish(cmd);

    }

    r.sleep();
    ros::spinOnce();
  }


}


