#pragma once

#include <ros/ros.h>
#include <cstdlib>

// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// using eigen lib
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "box.h"

namespace velodyne_pointcloud
{
/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR
{
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  uint16_t ring;                  ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

//Customed Point Struct for holding clustered points
namespace lidar_handler
{
/** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
struct PointXYZIRL
{
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  uint16_t ring;                  ///< laser ring number
  uint16_t label;                 ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace lidar_handler

#define SLRPointXYZIRL lidar_handler::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_handler::PointXYZIRL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(uint16_t, label, label))

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

class LiDARHandler
{

private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_;
  std::string point_topic_;

  int sensor_model_;
  double sensor_height_, clip_height_, min_distance_, max_distance_;
  int num_seg_ = 1;
  int num_iter_, num_lpr_;
  double th_seeds_, th_dist_;
  // Model parameter for ground plane fitting
  // The ground plane model is: ax+by+cz+d=0
  // Here normal:=[a,b,c], d=d
  // th_dist_d_ = threshold_dist - d
  float d_, th_dist_d_;
  MatrixXf normal_;
  float ClusterTolerance_,VoxelLeafSize_;
  int ClusterMinSize_,ClusterMaxSize_;

  tf2_ros::Buffer tf_tree;
  tf2_ros::TransformListener *tfListener;


  // pcl::PointCloud<VPoint>::Ptr g_seeds_pc(new pcl::PointCloud<VPoint>());
  // pcl::PointCloud<VPoint>::Ptr g_ground_pc(new pcl::PointCloud<VPoint>());
  // pcl::PointCloud<VPoint>::Ptr g_not_ground_pc(new pcl::PointCloud<VPoint>());
  // pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<SLRPointXYZIRL>());

  pcl::PointCloud<VPoint>::Ptr g_seeds_pc;
  pcl::PointCloud<VPoint>::Ptr g_ground_pc;
  pcl::PointCloud<VPoint>::Ptr g_not_ground_pc;
  pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc;

  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted);
  void post_process(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);
  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
  void clip_above(const pcl::PointCloud<VPoint>::Ptr in,
                  const pcl::PointCloud<VPoint>::Ptr out);
  void remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in,
                       const pcl::PointCloud<VPoint>::Ptr out);
  void rectangle_segment(const pcl::PointCloud<VPoint>::Ptr in,
                       const pcl::PointCloud<VPoint>::Ptr out);
  std::vector<pcl::PointCloud<VPoint>::Ptr> Clustering(pcl::PointCloud<VPoint>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
  Box BoundingBox(pcl::PointCloud<VPoint>::Ptr cluster);

public:
  LiDARHandler(ros::NodeHandle &nh);
  ~LiDARHandler();
  void Spin();
};
