#include <map_cleaner/MovingPointIdentification.hpp>

inline void MovingPointIdentification::getRangeImageCoordinate(
    const PointType &p, float &range, float &pos_h, float &pos_v) {
  range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  pos_h = std::atan2(p.y, p.x) * res_h_scale_;
  pos_v = std::asin(p.z / range) * res_v_scale_;
}

grid_map::GridMapPtr
MovingPointIdentification::buildRangeImage(const CloudType &input) {
  grid_map::GridMapPtr range_im(new grid_map::GridMap);
  range_im->setFrameId("map");
  range_im->setGeometry(
      grid_map::Length(fov_h_ * res_h_scale_, fov_v_ * res_v_scale_),
      range_im_res_, grid_map::Position(0.0, 0.0));
  range_im->add(layer_name_);

  grid_map::Matrix &layer = (*range_im)[layer_name_];
  layer.setConstant(std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < input.size(); i++) {
    const PointType &p = input[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }

    float range, pos_h, pos_v;
    getRangeImageCoordinate(p, range, pos_h, pos_v);

    grid_map::Index idx;
    if (!range_im->getIndex(grid_map::Position(pos_h, pos_v), idx)) {
      continue;
    }

    if (!std::isfinite(layer(idx[0], idx[1]))) {
      layer(idx[0], idx[1]) = range;
    } else {
      layer(idx[0], idx[1]) = std::min(layer(idx[0], idx[1]), range);
    }
  }

  return range_im;
}

void MovingPointIdentification::buildSubMap(
    const CloudType &input, const KDTreeNanoFlannPCL<PointType> &kdtree,
    const Eigen::Affine3f &lidar_pose, CloudType &submap,
    std::vector<int> &indices) {
  submap.clear();
  indices.clear();

  PointType center;
  center.x = lidar_pose.translation().x();
  center.y = lidar_pose.translation().y();
  center.z = lidar_pose.translation().z();

  std::vector<nanoflann::ResultItem<int, float>> nanoflann_res;
  kdtree.radiusSearch(center.data, lidar_range_sq_, nanoflann_res,
                      nanoflann::SearchParameters());

  submap.resize(nanoflann_res.size());
  indices.resize(nanoflann_res.size());
  for (int i = 0; i < submap.size(); ++i) {
    submap[i] = input[nanoflann_res[i].first];
    indices[i] = nanoflann_res[i].first;
  }
}

inline MovingPointIdentification::ComparisonResult
MovingPointIdentification::compareRange(const float scan_range,
                                        const float target_range) {
  if (!std::isfinite(scan_range) || !std::isfinite(target_range))
    return ComparisonResult::CASE_D;
  else if (target_range > scan_range + threshold_)
    return ComparisonResult::CASE_B;
  else if (target_range < scan_range - threshold_)
    // the target is closer than the scan point => it should be moving
    return ComparisonResult::CASE_C;
  else
    return ComparisonResult::CASE_A;
}

inline MovingPointIdentification::FusedResult
MovingPointIdentification::getFusedResult(const grid_map::GridMap &range_im,
                                          const grid_map::Matrix &layer,
                                          const grid_map::Position &target_pos,
                                          const float &target_range) {
  const grid_map::Size im_size = range_im.getSize();
  grid_map::Index target_idx;
  if (!range_im.getIndex(target_pos, target_idx))
    return FusedResult::OTHERWISE;

  int counter[4] = {0, 0, 0, 0};
  for (int d_v = -delta_v_; d_v <= delta_v_; ++d_v) {
    for (int d_h = -delta_h_; d_h <= delta_h_; ++d_h) {
      grid_map::Index idx(target_idx[0] + d_h, target_idx[1] + d_v);
      if (0 > idx[0]) {
        if (spinning_lidar_)
          idx[0] = im_size[0] + idx[0];
        else
          continue;
      }
      if (idx[0] >= im_size[0]) {
        if (spinning_lidar_)
          idx[0] = idx[0] - im_size[0];
        else
          continue;
      }
      if (0 > idx[1] || idx[1] >= im_size[1])
        continue;

      const float scan_range = layer(idx[0], idx[1]);
      ComparisonResult res = compareRange(scan_range, target_range);
      counter[(int)res]++;
    }
  }

  if (counter[0] != 0)
    return FusedResult::CASE_A; // static
  else if (counter[1] != 0)
    return FusedResult::OTHERWISE;
  else if (counter[2] != 0)
    return FusedResult::CASE_C; // dynamic

  return FusedResult::OTHERWISE;
}

void MovingPointIdentification::compareSubmapAndScan(
    const grid_map::GridMap &range_im, const CloudType &submap,
    const std::vector<int> &indices, std::vector<int> &vote_list_static,
    std::vector<int> &vote_list_dynamic) {
  const grid_map::Matrix &layer = range_im[layer_name_];

#pragma omp parallel for
  for (int i = 0; i < submap.size(); ++i) {
    float range, pos_h, pos_v;
    getRangeImageCoordinate(submap[i], range, pos_h, pos_v);
    const FusedResult res = getFusedResult(
        range_im, layer, grid_map::Position(pos_h, pos_v), range);
    if (res == FusedResult::CASE_A)
      vote_list_static[indices[i]]++;
    else if (res == FusedResult::CASE_C)
      vote_list_dynamic[indices[i]]++;
  }
}

grid_map::GridMapPtr global_range_im;

void MovingPointIdentification::publish(
    const DataLoaderBase::Frame &frame, const CloudType::ConstPtr &input,
    const std::vector<int> &indices, const std::vector<int> &vote_list_static,
    const std::vector<int> &vote_list_dynamic) {

  if (pub_ptr_ == nullptr)
    return;

  pcl::PointCloud<pcl::PointXYZRGB> vis_cloud;
  vis_cloud.reserve(indices.size() + frame.frame->size());

  grid_map::Matrix &layer = (*global_range_im)[layer_name_];

  auto inverseRangeImageCoordinate =
      [this](const PointType &p, const float range) -> pcl::PointXYZRGB {
    pcl::PointXYZRGB out;
    float tmp_range, pos_h, pos_v;
    this->getRangeImageCoordinate(p, tmp_range, pos_h, pos_v);
    out.x =
        std::cos(pos_h / res_h_scale_) * std::cos(pos_v / res_v_scale_) * range;
    out.y =
        std::sin(pos_h / res_h_scale_) * std::cos(pos_v / res_v_scale_) * range;
    out.z = std::sin(pos_v / res_v_scale_) * range;
    out.r = 255;
    out.g = 255;
    out.b = 255;
    return out;
  };

  // Publish raw lidar points (magenta) and range image (white)
  for (int i = 0; i < frame.frame->size(); i++) {
    const PointType &p = frame.frame->at(i);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    pcl::PointXYZRGB vis_p;
    vis_p.x = p.x;
    vis_p.y = p.y;
    vis_p.z = p.z;
    vis_p.r = 255;
    vis_p.g = 0;
    vis_p.b = 255;
    vis_cloud.push_back(vis_p);

    float range, pos_h, pos_v;
    getRangeImageCoordinate(p, range, pos_h, pos_v);

    grid_map::Index idx;
    if (!global_range_im->getIndex(grid_map::Position(pos_h, pos_v), idx)) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("moving_point_identification"),
                         "Cannot get index for visualization "
                             << pos_h << ", " << pos_v << " p=" << p.x << ","
                             << p.y << "," << p.z);
      continue;
    }

    if (std::isfinite(layer(idx[0], idx[1]))) {
      const float new_range = layer(idx[0], idx[1]);
      vis_cloud.push_back(inverseRangeImageCoordinate(p, new_range));
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(vis_cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.header.stamp = rclcpp::Clock().now();
  pub_ptr_->publish(cloud_msg);
}

MovingPointIdentification::MovingPointIdentification(
    const float voxel_leaf_size, const float res_h, const float res_v,
    const float fov_h, const float fov_v, const float lidar_range,
    const int delta_h, const int delta_v, const float threshold,
    const int frame_skip, const float submap_update_dist,
    const PublisherPtr pub_ptr, const std::string &frame_id, rclcpp::Node *node)
    : static_tf_br_(
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(node)) {
  vg_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  fov_h_ = fov_h;
  fov_v_ = fov_v;

  if (fov_h_ > M_PI * 2.0)
    fov_h_ = M_PI * 2.0;

  if (fov_v_ > M_PI)
    fov_v_ = M_PI;

  res_h_ = res_h;
  res_v_ = res_v;

  if ((M_PI * 2.0 - fov_h_) < res_h_)
    spinning_lidar_ = true;
  else
    spinning_lidar_ = false;

  if (res_h_ < res_v_) {
    res_h_scale_ = 1.0;
    res_v_scale_ = res_h_ / res_v_;
    range_im_res_ = res_h_;
  } else {
    res_h_scale_ = res_v_ / res_h_;
    res_v_scale_ = 1.0;
    range_im_res_ = res_v_;
  }

  lidar_range_sq_ = lidar_range * lidar_range;
  delta_h_ = delta_h;
  delta_v_ = delta_v;
  threshold_ = threshold;
  frame_skip_ = frame_skip;
  submap_update_dist_ = submap_update_dist;

  pub_ptr_ = pub_ptr;
  frame_id_ = frame_id;
}

bool MovingPointIdentification::compute(DataLoaderBase::Ptr &loader,
                                        const CloudType::ConstPtr &cloud,
                                        const PIndices::ConstPtr &in_indices,
                                        PIndices &static_indices,
                                        PIndices &dynamic_indices) {
  // (1) If cloud or loader is empty, return false
  if (cloud->empty() || loader->getSize() == 0 || in_indices->indices.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("moving_point_identification"),
                       "Input cloud or loader is empty."
                           << cloud->size() << ", " << loader->getSize() << ", "
                           << in_indices->indices.size());
    return false;
  }

  // (2) Create downsampled cloud and its index mapping
  CloudType::Ptr cloud_ds(new CloudType);
  std::vector<pcl::Indices> vg_indices;
  vg_.setInputCloud(cloud);
  vg_.setIndices(in_indices);
  vg_.filterWithOutputIndices(*cloud_ds, vg_indices);

  // (3) Define some variables which will be used in the loop
  std::vector<int> vote_list_static;
  vote_list_static.resize(cloud_ds->size(), 0);
  std::vector<int> vote_list_dynamic;
  vote_list_dynamic.resize(cloud_ds->size(), 0);

  PointCloudAdaptor<PointType> adapter;
  adapter.cloud_ptr_ = cloud_ds;
  KDTreeNanoFlannPCL<PointType> kdtree(3, adapter);
  kdtree.buildIndex();

  CloudType::Ptr submap(new CloudType);
  CloudType::Ptr lidar_coordinate_submap(new CloudType);
  std::vector<int> submap_indices;
  submap_indices.reserve(cloud_ds->size());
  Eigen::Affine3f last_lidar_pose = Eigen::Affine3f::Identity();

  // (4) Iterate all submaps and compare with range image
  for (int i = 0; i < loader->getSize(); ++i) {
    // (4-1) Load frame and skip if empty
    DataLoaderBase::Frame frame = loader->loadFrame(i);
    if (frame.frame->empty()) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("moving_point_identification"),
                         "Frame " << frame.idx + 1 << " is empty.");
      continue;
    }

    // (4-2) Skip for reduce computation time. If frame_skip_ = 1, it means
    // the computation cost wil be halved.
    if (i % (frame_skip_ + 1) != 0)
      continue;

    // (4-3) Build range image from the not-downsampled current scan
    grid_map::GridMapPtr range_im = buildRangeImage(*frame.frame);

    // (4-4) Build submap based on the current pose when the accumulated
    // motion exceeds a threshold. NOTE: The submap is the partial pointcloud
    // of the downsampled whole map which is within lidar_range_ from the
    // current lidar pose.
    const Eigen::Affine3f lidar_pose =
        Eigen::Translation3f(frame.t) * frame.r.matrix();
    if (submap->empty() || (last_lidar_pose.translation() - frame.t).norm() >
                               submap_update_dist_) {
      buildSubMap(*adapter.cloud_ptr_, kdtree, lidar_pose, *submap,
                  submap_indices);
      last_lidar_pose = lidar_pose;
    }

    // (4-5) Inrementally vote static or dynamic by comparing range image and
    // submap.
    pcl::transformPointCloud(*submap, *lidar_coordinate_submap,
                             lidar_pose.inverse());
    compareSubmapAndScan(*range_im, *lidar_coordinate_submap, submap_indices,
                         vote_list_static, vote_list_dynamic);

    if (i % 10 == 0) {
      global_range_im = range_im;
      publish(frame, cloud_ds, submap_indices, vote_list_static,
              vote_list_dynamic);

      RCLCPP_INFO_STREAM(rclcpp::get_logger("moving_point_indentification"),
                         "Compute Moving Point Identification: "
                             << i + 1 << " / " << loader->getSize());
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("moving_point_indentification"),
                     "Compute Moving Point Identification: "
                         << loader->getSize() << " / " << loader->getSize());

  // (5) Push back results depending on votes
  static_indices.indices.clear();
  dynamic_indices.indices.clear();
  for (int i = 0; i < cloud_ds->size(); i++) {
    PIndices *dst_indices;
    // If both of static and dynamic votes are the same, it will be
    // static.
    if (vote_list_dynamic[i] <= vote_list_static[i])
      dst_indices = &static_indices;
    else
      dst_indices = &dynamic_indices;

    for (int j = 0; j < vg_indices[i].size(); ++j) {
      dst_indices->indices.push_back(vg_indices[i][j]);
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("moving_point_indentification"),
                     "Finish Moving Point Identification");
  static_indices.indices.shrink_to_fit();
  dynamic_indices.indices.shrink_to_fit();

  return true;
}
