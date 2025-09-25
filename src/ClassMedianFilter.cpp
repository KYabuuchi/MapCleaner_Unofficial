#include "map_cleaner/ClassMedianFilter.hpp"
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>

ClassMedianFilter::ClassMedianFilter(const int neightbor_size)
    : neighbor_size_(neightbor_size) {}

void ClassMedianFilter::filter(const CloudType::Ptr &cloud,
                               const PIndices::ConstPtr &static_indices,
                               const PIndices::ConstPtr &dynamic_indices,
                               PIndices::Ptr out_static_indices,
                               PIndices::Ptr out_dynamic_indices) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("class_median_filter"),
                     "ClassMedianFilter::filter"
                         << "neightbor_size_: " << neighbor_size_);

  pcl::PointCloud<pcl::PointXYZL> labeled_cloud;
  for (const auto &index : static_indices->indices) {
    pcl::PointXYZL p;
    p.getVector4fMap() = cloud->points[index].getVector4fMap();
    p.label = 1; // static
    labeled_cloud.points.push_back(p);
  }
  for (const auto &index : dynamic_indices->indices) {
    pcl::PointXYZL p;
    p.getVector4fMap() = cloud->points[index].getVector4fMap();
    p.label = 2; // dynamic
    labeled_cloud.points.push_back(p);
  }

  // Create KD-Tree
  PointCloudAdaptor<pcl::PointXYZL> adapter;
  adapter.cloud_ptr_ = labeled_cloud.makeShared();
  KDTreeNanoFlannPCL<pcl::PointXYZL> kdtree(3, adapter);
  kdtree.buildIndex();

  std::vector<int> ret_index(neighbor_size_);
  std::vector<float> out_dist_sqr(neighbor_size_);

  PIndices::Ptr tmp_out_static_indices(new PIndices);
  PIndices::Ptr tmp_out_dynamic_indices(new PIndices);

  auto process = [&](size_t index) -> void {
    pcl::PointXYZL query_point;
    query_point.getVector4fMap() = cloud->points[index].getVector4fMap();

    kdtree.knnSearch(query_point.data, neighbor_size_, ret_index.data(),
                     out_dist_sqr.data());

    int count_static = 0;
    int count_dynamic = 0;
    for (size_t i = 0; i < ret_index.size(); i++) {
      if (labeled_cloud.points[ret_index[i]].label == 1)
        count_static++;
      else if (labeled_cloud.points[ret_index[i]].label == 2)
        count_dynamic++;
    }

    if (count_static > count_dynamic) {
      tmp_out_static_indices->indices.push_back(index);
    } else {
      tmp_out_dynamic_indices->indices.push_back(index);
    }
  };

  for (const auto &index : static_indices->indices) {
    process(index);
  }
  for (const auto &index : dynamic_indices->indices) {
    process(index);
  }

  out_static_indices->indices = tmp_out_static_indices->indices;
  out_dynamic_indices->indices = tmp_out_dynamic_indices->indices;
}