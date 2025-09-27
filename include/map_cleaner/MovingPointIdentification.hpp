#pragma once

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <map_cleaner/DataLoader.hpp>
#include <map_cleaner/utils.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <voxel_grid_large.h>

class MovingPointIdentification {
public:
  typedef std::shared_ptr<MovingPointIdentification> Ptr;
  typedef std::shared_ptr<const MovingPointIdentification> ConstPtr;

private:
  const std::string layer_name_ = "range";

  float lidar_range_sq_;
  float fov_h_, fov_v_;
  float res_h_, res_v_;
  float res_h_scale_, res_v_scale_, range_im_res_;
  int delta_h_, delta_v_;
  bool spinning_lidar_;
  int frame_skip_;
  float threshold_;
  float submap_update_dist_;
  pcl::VoxelGridLarge<PointType> vg_;

  PublisherPtr pub_ptr_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_br_;
  std::string frame_id_;

  enum class ComparisonResult {
    CASE_A = 0,
    CASE_B = 1,
    CASE_C = 2,
    CASE_D = 3
  };
  enum class FusedResult { CASE_A, CASE_C, OTHERWISE };

  inline void getRangeImageCoordinate(const PointType &p, float &range,
                                      float &pos_h, float &pos_v);

  grid_map::GridMapPtr buildRangeImage(const CloudType &input);

  void buildSubMap(const CloudType &input,
                   const KDTreeNanoFlannPCL<PointType> &kdtree,
                   const Eigen::Affine3f &lidar_pose, CloudType &submap,
                   std::vector<int> &indices);

  inline ComparisonResult compareRange(const float scan_range,
                                       const float target_range);

  [[maybe_unused]] inline FusedResult getFusedResult(
      const grid_map::GridMap &range_im, const grid_map::Matrix &layer,
      const grid_map::Position &target_pos, const float &target_range);

  void compareSubmapAndScan(const grid_map::GridMap &range_im,
                            const CloudType &submap,
                            const std::vector<int> &indices,
                            std::vector<int> &vote_list_static,
                            std::vector<int> &vote_list_dynamic);

  void publish(const DataLoaderBase::Frame &frame,
               const CloudType::ConstPtr &input,
               const std::vector<int> &indices,
               const std::vector<int> &vote_list_static,
               const std::vector<int> &vote_list_dynamic);

public:
  MovingPointIdentification(const float voxel_leaf_size, const float res_h,
                            const float res_v, const float fov_h,
                            const float fov_v, const float lidar_range,
                            const int delta_h, const int delta_v,
                            const float threshold, const int frame_skip,
                            const float submap_update_dist,
                            const PublisherPtr pub_ptr = nullptr,
                            const std::string &frame_id = "map",
                            rclcpp::Node *node = nullptr);

  bool compute(DataLoaderBase::Ptr &loader, const CloudType::ConstPtr &cloud,
               const PIndices::ConstPtr &in_indices, PIndices &static_indices,
               PIndices &dynamic_indices);
};
