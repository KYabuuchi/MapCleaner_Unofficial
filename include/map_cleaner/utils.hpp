#pragma once

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <nanoflann/nanoflann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

#define FRAME_ID "map"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    PublisherPtr;
typedef pcl::PointIndices PIndices;

namespace grid_map {
typedef std::shared_ptr<grid_map::GridMap> GridMapPtr;
typedef std::shared_ptr<const grid_map::GridMap> GridMapConstPtr;
} // namespace grid_map

template <typename PointT> struct PointCloudAdaptor {
  typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;
  CloudConstPtr cloud_ptr_;

  inline size_t kdtree_get_point_count() const { return cloud_ptr_->size(); }
  inline float kdtree_get_pt(const size_t idx, int dim) const {
    switch (dim) {
    case 0:
      return cloud_ptr_->points[idx].x;
    case 1:
      return cloud_ptr_->points[idx].y;
    case 2:
      return cloud_ptr_->points[idx].z;
    default:
      return 0.0;
    }
  }
  template <class BBOX> bool kdtree_get_bbox(BBOX &) const { return false; }
};

template <typename PointT>
using KDTreeNanoFlannPCL = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor<PointT>>,
    PointCloudAdaptor<PointT>, 3, int>;