#include "main_server.h"

bool OverRange(geometry_msgs::Point a, geometry_msgs::Point b, double rangepower)
{
  if((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)>rangepower)
    return true;
  else
    return false;
}
void car_pose_update(const geometry_msgs::PoseStampedPtr car_pose)
{
  if(isInit)
  {
    car_odo.header.frame_id="uav";
    car_odo.pose.pose.position.x=car_pose->pose.position.x-init_x;
    car_odo.pose.pose.position.y=car_pose->pose.position.y-init_y;
    car_odo.pose.pose.position.z=1.5;
    car_odo.pose.pose.orientation = car_pose->pose.orientation;
    car_odo.twist.twist.linear.x = 0.0;
    car_odo.twist.twist.linear.y = 0.0;
    car_odo.twist.twist.linear.z = 0.0;
    car_odo.twist.twist.angular.x = 0.0;
    car_odo.twist.twist.angular.y = 0.0;
    car_odo.twist.twist.angular.z = 0.0;
//    car_odo.twist.twist.linear.x = gn.Ve*0.95138+gn.Vn*0.308;
//    car_odo.twist.twist.linear.y = gn.Vn*0.95137-gn.Ve*0.308;
    car_odo_pub_.publish(car_odo);

    trans_w_b.transform.translation.x = car_odo.pose.pose.position.x;
    trans_w_b.transform.translation.y = car_odo.pose.pose.position.y;
    trans_w_b.transform.translation.z = 0.4;

    trans_w_b.transform.rotation = car_pose->pose.orientation;

    trans_m_w.header.stamp = ros::Time::now();
    trans_w_b.header.stamp = ros::Time::now();
    trans_b_s.header.stamp = ros::Time::now();

    br->sendTransform(trans_m_w);
    br->sendTransform(trans_w_b);
    br->sendTransform(trans_b_s);
  }
  else
  {
    init_x=car_pose->pose.position.x;
    init_y=car_pose->pose.position.y;

    trans_m_w.transform.translation.x = init_x;
    trans_m_w.transform.translation.y = init_y;
    trans_m_w.transform.translation.z = 0;

    isInit=true;
  }

  if(is_global_path_valid && (!OverRange(global_path_latest[nowGoal].pose.position,car_pose->pose.position,reGoalDisPOwer)))
  {
    for(size_t i=lastGoal;i<global_path_latest.size();++i)
    {
      if(OverRange(global_path_latest[i].pose.position,car_pose->pose.position,goalDisPower)
         || i == (global_path_latest.size()-1))
      {
        lastGoal = nowGoal = i;
        geometry_msgs::PoseStamped goal_world;
        goal_world.header.frame_id = "world";
        goal_world.pose.position.x = global_path_latest[i].pose.position.x-init_x;
        goal_world.pose.position.y = global_path_latest[i].pose.position.y-init_y;
        goal_world.pose.position.z = 1.5;
        nav_msgs::Path goal;
        goal.poses.push_back(goal_world);
        goal_pub_.publish(goal);
        break;
      }
    }
  }
}

void marker_to_path(const visualization_msgs::MarkerPtr traj)
{
  nav_msgs::Path local_path;
  local_path.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;

  for(auto &t : traj->points)
  {
    pose.pose.position = t;
    local_path.poses.push_back(pose);
  }
  local_path.header.stamp = ros::Time::now();

  traj_pub_.publish(local_path);
}

void global_path_update(const nav_msgs::PathPtr gloabal_path)
{
  is_global_path_valid = false;
  global_path_latest = gloabal_path->poses;
  std::reverse(global_path_latest.begin(),global_path_latest.end());
  lastGoal = 0;
  nowGoal = 0;
  if(global_path_latest.size()>1)
  {
    is_global_path_valid = true;
    for(size_t i=1;i<global_path_latest.size();++i)
    {
      if(OverRange(global_path_latest[i].pose.position,global_path_latest[0].pose.position,goalDisPower)
         || i == (global_path_latest.size()-1))
      {
        geometry_msgs::PoseStamped goal_world;
        goal_world.header.frame_id = "world";
        goal_world.pose.position.x = global_path_latest[i].pose.position.x-init_x;
        goal_world.pose.position.y = global_path_latest[i].pose.position.y-init_y;
        goal_world.pose.position.z = 1.5;
        nav_msgs::Path goal;
        goal.poses.push_back(goal_world);
        goal_pub_.publish(goal);
        lastGoal = nowGoal = i;
        break;
      }
    }
  }

}

void fmm_path_update(const visualization_msgs::MarkerArrayPtr fmm_path)
{
  fmm_path_latest.clear();
  is_fmm_path_valid = false;
  geometry_msgs::PoseStamped tmp;
  for(size_t i = 0;i<fmm_path->markers.size();++i)
  {
    tmp.pose.position = fmm_path->markers[i].pose.position;
    fmm_path_latest.push_back(tmp);
  }
  is_fmm_path_valid = true;
}

void error_handler(const std_msgs::StringPtr err)
{
  if(err->data == "solver_error")
  {
    if(is_fmm_path_valid)
    {
      nav_msgs::Path fmm_path;
      fmm_path.header.frame_id = "world";
      fmm_path.header.stamp = ros::Time::now();
      fmm_path.poses = fmm_path_latest;
      traj_pub_.publish(fmm_path);
    }
  }
  else if(err->data == "goal_error")
  {
    if(is_global_path_valid)
    {
      if(nowGoal<global_path_latest.size()-1)
      {
        lastGoal = nowGoal +=1;
      }
      else
      {
        lastGoal = nowGoal -=1;
      }
      geometry_msgs::PoseStamped goal_world;
      goal_world.header.frame_id = "world";
      goal_world.pose.position.x = global_path_latest[nowGoal].pose.position.x-init_x;
      goal_world.pose.position.y = global_path_latest[nowGoal].pose.position.y-init_y;
      goal_world.pose.position.z = 1.5;
      nav_msgs::Path goal;
      goal.poses.push_back(goal_world);
      goal_pub_.publish(goal);
    }
  }
}


int main(int argc , char** argv)
{
  ros::init(argc,argv,"main_server");
  ros::NodeHandle nh;

  car_pose_sub_ = nh.subscribe("/car_pose",1000,car_pose_update);
  car_odo_pub_ = nh.advertise<nav_msgs::Odometry>("/car_odo",10);

  traj_sub_ = nh.subscribe("/b_traj_node/trajectory_vis",1000,marker_to_path);
  traj_pub_ = nh.advertise<nav_msgs::Path>("/local_path",10);

  path_sub_ = nh.subscribe("/global_path",1000,global_path_update);
  goal_pub_ = nh.advertise<nav_msgs::Path>("/waypoints",10);

  fmm_path_sub_ = nh.subscribe("/b_traj_node/path_vis",1000,fmm_path_update);
  error_sub_ = nh.subscribe("/error_msg",1000,error_handler);

  br = new tf2_ros::TransformBroadcaster;

  trans_m_w.header.frame_id = "map";
  trans_m_w.child_frame_id = "world";
  trans_w_b.header.frame_id = "world";
  trans_w_b.child_frame_id = "base_link";
  trans_b_s.header.frame_id = "base_link";
  trans_b_s.child_frame_id = "sensor_frame";

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(0);
  trans_m_w.transform.rotation = q;
  trans_b_s.transform.rotation = q;

  trans_b_s.transform.translation.x = 0.5;
  trans_b_s.transform.translation.z = 2.3;


  ros::spin();
}
