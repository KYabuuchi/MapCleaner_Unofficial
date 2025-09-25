#include <map_cleaner/BGKFilter.hpp>
#include <map_cleaner/DataLoader.hpp>
#include <map_cleaner/DivideByTerrain.hpp>
#include <map_cleaner/GridMapBuilder.hpp>
#include <map_cleaner/GroundSegmentation.hpp>
#include <map_cleaner/MedianFilter.hpp>
#include <map_cleaner/MovingPointIdentification.hpp>
#include <map_cleaner/TrajectoryFilter.hpp>
#include <map_cleaner/VarianceFilter.hpp>
#include <map_cleaner/utils.hpp>
#include <pcd_io_with_indices.h>
#include <rclcpp/rclcpp.hpp>
#include <voxel_grid_large.h>

// #define PUBLISH_GRID_MAP

class MapCleaner : public rclcpp::Node {

  std::string save_dir_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_static_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_below_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_other_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_;

#ifdef PUBLISH_GRID_MAP
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
#endif

  DataLoaderBase::Ptr loader_;
  GroundSegmentation::Ptr ground_seg_;
  GridMapBuilder::Ptr grid_map_builder_;
  VarianceFilter::Ptr variance_filter_;
  BGKFilter::Ptr first_bgk_filter_;
  TrajectoryFilter::Ptr trajectory_filter_;
  BGKFilter::Ptr second_bgk_filter_;
  MedianFilter::Ptr median_filter_;
  DivideByTerrain::Ptr divide_by_terrain_;
  MovingPointIdentification::Ptr moving_point_identification_;

  float resolution_;
  std::string frame_id_;

  pcl::VoxelGridLarge<PointType> vis_vg_;

  void publishCloud(
      const CloudType::ConstPtr &cloud, const PIndices::ConstPtr &indices,
      const std::string frame_id,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub) {
    CloudType::Ptr vis_cloud(new CloudType);
    if (indices != nullptr) {
      vis_vg_.setInputCloud(cloud);
      vis_vg_.setIndices(indices);
      vis_vg_.filter(*vis_cloud);
    } else {
      *vis_cloud = *cloud;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*vis_cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    pub->publish(cloud_msg);
  }

#ifdef PUBLISH_GRID_MAP
  void publishGridMap(
      const grid_map::GridMap &grid,
      const std::vector<std::string> &publish_layer,
      rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr &pub) {
    grid_map::GridMap vis_grid;
    vis_grid.setFrameId(grid.getFrameId());
    vis_grid.setGeometry(grid.getLength(), grid.getResolution(),
                         grid.getPosition());
    for (int i = 0; i < publish_layer.size(); i++) {
      if (grid.exists(publish_layer[i])) {
        vis_grid.add(publish_layer[i], grid[publish_layer[i]]);
      }
    }

    grid_map_msgs::GridMap grid_msg;
    grid_map::GridMapRosConverter::toMessage(vis_grid, grid_msg);
    pub.publish(grid_msg);
  }
#endif

  CloudType::Ptr gridMap2Cloud(const grid_map::GridMap &grid,
                               const std::string &layer_name,
                               const std::string &intensity_layer_name) {
    CloudType::Ptr res(new CloudType);
    if (!grid.exists(layer_name))
      return res;
    const grid_map::Matrix &layer = grid[layer_name];

    const grid_map::Matrix *intensity_layer_ptr = nullptr;
    if (grid.exists(intensity_layer_name))
      intensity_layer_ptr = &(grid[intensity_layer_name]);

    res->reserve(grid.getSize()[0] * grid.getSize()[1]);
    for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
      int i = it.getLinearIndex();
      float z = layer(i);
      if (!std::isfinite(z))
        continue;

      grid_map::Position pos;
      grid.getPosition(*it, pos);

      PointType p;
      p.x = pos[0];
      p.y = pos[1];
      p.z = z;

      if (intensity_layer_ptr != nullptr) {
        float intensity = (*intensity_layer_ptr)(i);
        if (std::isfinite(intensity))
          p.intensity = intensity;
        else
          p.intensity = 0;
      }

      res->push_back(p);
    }

    return res;
  }

  bool saveCloud(const std::string &filename, const CloudType &cloud,
                 const PIndices::ConstPtr indices_ptr = nullptr) {
    try {
      pcl::PCDWriterWithIndices writer;
      if (indices_ptr == nullptr) {
        PIndices dummy_indices;
        dummy_indices.indices.resize(cloud.size());
        for (int i = 0; i < cloud.size(); i++)
          dummy_indices.indices[i] = i;
        writer.writeBinaryWithIndices(filename, cloud, dummy_indices);
      } else {
        writer.writeBinaryWithIndices(filename, cloud, *indices_ptr);
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  void initialize() {

    rclcpp::QoS qos = rclcpp::QoS{1}.transient_local().reliable();

    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "ground_cloud", qos);
    pub_static_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "static_cloud", qos);
    pub_dynamic_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "dynamic_cloud", qos);
    pub_ground_below_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "ground_below_cloud", qos);
    pub_other_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "other_cloud", qos);
    pub_terrain_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "terrain_cloud", qos);
#ifdef PUBLISH_GRID_MAP
    pub_grid_map_ =
        this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", qos);
#endif

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_ptr_in_proc_vis_;
    pub_ptr_in_proc_vis_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("in_process",
                                                              qos);

    frame_id_ = this->declare_parameter<std::string>("map_cleaner.frame_id");
    save_dir_ = this->declare_parameter<std::string>("map_cleaner.save_dir");

    if (save_dir_.back() != '/') {
      save_dir_ += '/';
    }

    float vis_vg_res;
    vis_vg_res =
        this->declare_parameter<float>("map_cleaner.visualization_vg_res");
    vis_vg_.setLeafSize(vis_vg_res, vis_vg_res, vis_vg_res);

    bool in_process_vis;
    in_process_vis =
        this->declare_parameter<bool>("map_cleaner.in_process_visualization");

    // Loader
    std::string pcds_dir, pose_file, calibration_file, format;
    int start, end;
    pcds_dir = this->declare_parameter<std::string>("loader.pcds_dir");
    RCLCPP_INFO(this->get_logger(), "PCDs Dir: %s", pcds_dir.c_str());
    if (pcds_dir.back() != '/')
      pcds_dir += '/';
    pose_file = this->declare_parameter<std::string>("loader.pose_file");
    calibration_file =
        this->declare_parameter<std::string>("loader.kitti_calibration_file");
    start = this->declare_parameter<int>("loader.start");
    end = this->declare_parameter<int>("loader.end");
    format = this->declare_parameter<std::string>("loader.format");

    if (format == "kitti") {
      loader_.reset(new KittiFormatLoader());
      std::dynamic_pointer_cast<KittiFormatLoader>(loader_)
          ->loadKittiCalibration(calibration_file);
      loader_->loadFrameInfo(pcds_dir, pose_file, start, end);
    } else if (format == "glim") {
      loader_.reset(new GLIMFormatLoader());
      loader_->loadFrameInfo(pcds_dir, pose_file, start, end);
    } else {
      loader_.reset(new ERASORFormatLoader());
      loader_->loadFrameInfo(pcds_dir, pose_file, start, end);
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Loaded Frames: " << loader_->getSize());

    if (loader_->getSize() == 0)
      exit(1);

    // PatchWorkpp
    bool use_voxel_grid;
    float seg_voxel_leaf_size;
    int seg_frame_skip;
    use_voxel_grid =
        this->declare_parameter<bool>("ground_segmentation.use_voxel_grid");
    seg_voxel_leaf_size =
        this->declare_parameter<float>("ground_segmentation.voxel_leaf_size");
    seg_frame_skip =
        this->declare_parameter<int>("ground_segmentation.frame_skip");

    patchwork::Params patchwork_params;
    patchwork_params.verbose =
        this->declare_parameter<bool>("patchworkpp.verbose");
    patchwork_params.enable_RNR =
        this->declare_parameter<bool>("patchworkpp.enable_rnr");
    patchwork_params.enable_RVPF =
        this->declare_parameter<bool>("patchworkpp.enable_rvpf");
    patchwork_params.enable_TGR =
        this->declare_parameter<bool>("patchworkpp.enable_tgr");
    patchwork_params.num_iter =
        this->declare_parameter<int>("patchworkpp.num_iter");
    patchwork_params.num_lpr =
        this->declare_parameter<int>("patchworkpp.num_lpr");
    patchwork_params.num_min_pts =
        this->declare_parameter<int>("patchworkpp.num_min_pts");
    patchwork_params.num_zones =
        this->declare_parameter<int>("patchworkpp.num_zones");
    patchwork_params.num_rings_of_interest =
        this->declare_parameter<int>("patchworkpp.num_rings_of_interest");
    patchwork_params.RNR_ver_angle_thr =
        this->declare_parameter<double>("patchworkpp.rnr_ver_angle_thr");
    patchwork_params.RNR_intensity_thr =
        this->declare_parameter<double>("patchworkpp.rnr_intensity_thr");
    patchwork_params.sensor_height =
        this->declare_parameter<double>("patchworkpp.sensor_height");
    patchwork_params.th_seeds =
        this->declare_parameter<double>("patchworkpp.th_seeds");
    patchwork_params.th_dist =
        this->declare_parameter<double>("patchworkpp.th_dist");
    patchwork_params.th_seeds_v =
        this->declare_parameter<double>("patchworkpp.th_seeds_v");
    patchwork_params.th_dist_v =
        this->declare_parameter<double>("patchworkpp.th_dist_v");
    patchwork_params.max_range =
        this->declare_parameter<double>("patchworkpp.max_range");
    patchwork_params.min_range =
        this->declare_parameter<double>("patchworkpp.min_range");
    patchwork_params.uprightness_thr =
        this->declare_parameter<double>("patchworkpp.uprightness_thr");
    patchwork_params.adaptive_seed_selection_margin =
        this->declare_parameter<double>(
            "patchworkpp.adaptive_seed_selection_margin");
    patchwork_params.num_sectors_each_zone =
        this->declare_parameter<std::vector<long>>(
            "patchworkpp.num_sectors_each_zone");
    patchwork_params.num_rings_each_zone =
        this->declare_parameter<std::vector<long>>(
            "patchworkpp.num_rings_each_zone");
    patchwork_params.max_flatness_storage =
        this->declare_parameter<int>("patchworkpp.max_flatness_storage");
    patchwork_params.max_elevation_storage =
        this->declare_parameter<int>("patchworkpp.max_elevation_storage");
    patchwork_params.elevation_thr =
        this->declare_parameter<std::vector<double>>(
            "patchworkpp.elevation_thr");
    patchwork_params.flatness_thr =
        this->declare_parameter<std::vector<double>>(
            "patchworkpp.flatness_thr");
    if (in_process_vis)
      ground_seg_.reset(new GroundSegmentation(
          patchwork_params, use_voxel_grid, seg_voxel_leaf_size, seg_frame_skip,
          pub_ptr_in_proc_vis_, frame_id_, this));
    else
      ground_seg_.reset(new GroundSegmentation(patchwork_params, use_voxel_grid,
                                               seg_voxel_leaf_size,
                                               seg_frame_skip));

    // GridMapBuilder
    resolution_ = this->declare_parameter<float>("grid_map_builder.grid_res");
    grid_map_builder_.reset(new GridMapBuilder(resolution_, frame_id_));
    RCLCPP_INFO_STREAM(this->get_logger(), "GridMapBuilder: initialized ");

    // VarianceFilter
    float variance_th;
    int variance_min_num;
    variance_th = this->declare_parameter<float>("variance_filter.variance_th");
    variance_min_num = this->declare_parameter<int>("variance_filter.min_num");
    variance_filter_.reset(new VarianceFilter(variance_th, variance_min_num));

    // First BGKFilter
    double bgk1_kernel_radius, bgk1_lambda, bgk1_sigma;
    int bgk1_min_num;
    bgk1_kernel_radius =
        this->declare_parameter<double>("first_bgk.kernel_radius");
    bgk1_lambda = this->declare_parameter<double>("first_bgk.lambda");
    bgk1_sigma = this->declare_parameter<double>("first_bgk.sigma");
    bgk1_min_num = this->declare_parameter<int>("first_bgk.min_num");
    first_bgk_filter_.reset(
        new BGKFilter(bgk1_kernel_radius, bgk1_lambda, bgk1_sigma, bgk1_min_num,
                      "variance_filtered", "first_bgk_filtered"));
    RCLCPP_INFO_STREAM(this->get_logger(), "First BGKFilter: initialized ");

    // TrajectoryFilter
    double normal_th_deg;
    normal_th_deg =
        this->declare_parameter<double>("trajectory_filter.normal_th_deg");
    trajectory_filter_.reset(
        new TrajectoryFilter((normal_th_deg / 180.0) * M_PI,
                             "first_bgk_filtered", "trajectory_filtered"));
    RCLCPP_INFO_STREAM(this->get_logger(), "TrajectoryFilter: initialized ");

    // Second BGKFilter
    double bgk2_kernel_radius, bgk2_lambda, bgk2_sigma;
    int bgk2_min_num;
    bgk2_kernel_radius =
        this->declare_parameter<double>("second_bgk.kernel_radius");
    bgk2_lambda = this->declare_parameter<double>("second_bgk.lambda");
    bgk2_sigma = this->declare_parameter<double>("second_bgk.sigma");
    bgk2_min_num = this->declare_parameter<int>("second_bgk.min_num");
    second_bgk_filter_.reset(
        new BGKFilter(bgk2_kernel_radius, bgk2_lambda, bgk2_sigma, bgk2_min_num,
                      "trajectory_filtered", "second_bgk_filtered"));

    // MedianFilter
    bool enable_median;
    int median_kernel_size;
    enable_median = this->declare_parameter<bool>("median.enable");
    median_kernel_size = this->declare_parameter<int>("median.kernel_size");
    if (enable_median) {
      median_filter_.reset(new MedianFilter(
          median_kernel_size, "second_bgk_filtered", "elevation"));
    } else {
      median_filter_ = nullptr;
    }

    // DivideByTerrain
    bool on_terrain_only;
    double terrain_th;
    on_terrain_only =
        this->declare_parameter<bool>("divide_by_terrain.on_terrain_only");
    terrain_th = this->declare_parameter<double>("divide_by_terrain.threshold");
    const double height_threshold =
        this->declare_parameter<double>("divide_by_terrain.height_threshold");
    divide_by_terrain_.reset(new DivideByTerrain(
        terrain_th, on_terrain_only, height_threshold, "elevation"));

    // MovingObjectIdentification
    double voxel_leaf_size, fov_h, fov_v, res_h, res_v, lidar_range,
        range_distance_th, submap_update_dist;
    int delta_h, delta_v, frame_skip;
    voxel_leaf_size = this->declare_parameter<double>(
        "moving_point_identification.voxel_leaf_size");
    fov_h = this->declare_parameter<double>(
        "moving_point_identification.fov_h_deg");
    fov_h = (fov_h / 180.0) * M_PI;
    fov_v = this->declare_parameter<double>(
        "moving_point_identification.fov_v_deg");
    fov_v = (fov_v / 180.0) * M_PI;
    res_h = this->declare_parameter<double>(
        "moving_point_identification.res_h_deg");
    res_h = (res_h / 180.0) * M_PI;
    res_v = this->declare_parameter<double>(
        "moving_point_identification.res_v_deg");
    res_v = (res_v / 180.0) * M_PI;
    lidar_range = this->declare_parameter<double>(
        "moving_point_identification.lidar_range");
    delta_h =
        this->declare_parameter<int>("moving_point_identification.delta_h");
    delta_v =
        this->declare_parameter<int>("moving_point_identification.delta_v");
    range_distance_th = this->declare_parameter<double>(
        "moving_point_identification.threshold");
    frame_skip =
        this->declare_parameter<int>("moving_point_identification.frame_skip");
    submap_update_dist = this->declare_parameter<double>(
        "moving_point_identification.submap_update_dist");

    if (in_process_vis) {
      moving_point_identification_.reset(new MovingPointIdentification(
          voxel_leaf_size, res_h, res_v, fov_h, fov_v, lidar_range, delta_h,
          delta_v, range_distance_th, frame_skip, submap_update_dist,
          pub_ptr_in_proc_vis_, frame_id_, this));
    } else {
      moving_point_identification_.reset(new MovingPointIdentification(
          voxel_leaf_size, res_h, res_v, fov_h, fov_v, lidar_range, delta_h,
          delta_v, range_distance_th, frame_skip, submap_update_dist));
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "pcds_dir: " << pcds_dir);
    RCLCPP_INFO_STREAM(this->get_logger(), "pose_file: " << pose_file);
    RCLCPP_INFO_STREAM(this->get_logger(), "save_dir: " << save_dir_);
  }

public:
  MapCleaner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("map_cleaner", options) {
    initialize();
    exec();
  }

  void exec() {

    // (1) Divide Point Cloud into Ground and Non-ground
    // (1-1) Ground Segmentation
    CloudType::Ptr cloud(new CloudType);
    PIndices::Ptr initial_ground_indices(new PIndices);
    PIndices::Ptr nonground_indices(new PIndices);
    ground_seg_->compute(loader_, *cloud, *initial_ground_indices,
                         *nonground_indices);
    RCLCPP_INFO_STREAM(this->get_logger(), "Finished: GroundSegmentation");

    // (1-2) Grid Map Based Terrain Extraction
    grid_map::GridMapPtr grid_map_ptr =
        grid_map_builder_->compute(*cloud, *initial_ground_indices);
    if (grid_map_ptr == nullptr) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed: ground_cloud is empty.");
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finished: GridMapBuilder");

    // (1-3) Grid Map Filtering
    if (!variance_filter_->compute(*grid_map_ptr)) {
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finished: VarianceFilter");

    // (1-4) first BGK Filtering
    if (!first_bgk_filter_->compute(*grid_map_ptr)) {
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finished: First BGKFilter");

    // (1-5) Trajectory Filtering
    if (!trajectory_filter_->compute(*grid_map_ptr, loader_)) {
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finished: TrajectoryFilter");

    // (1-6) second BGK Filtering
    if (!second_bgk_filter_->compute(*grid_map_ptr)) {
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finished: Second BGKFilter");

    // (1-7) Median Filtering if enabled
    if (median_filter_ != nullptr) {
      if (!median_filter_->compute(*grid_map_ptr)) {
        return;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Finished: MedianFilter");
    } else {
      grid_map_ptr->add("elevation", (*grid_map_ptr)["second_bgk_filtered"]);
      grid_map_ptr->erase("second_bgk_filtered");
    }

#ifdef PUBLISH_GRID_MAP
    publishGridMap(*grid_map_ptr, {"elevation", "intensity"}, pub_grid_map_);
#endif

    // (1-8) Publish Terrain Cloud
    CloudType::Ptr terrain_pcd =
        gridMap2Cloud(*grid_map_ptr, "elevation", "intensity");
    publishCloud(terrain_pcd, nullptr, frame_id_, pub_terrain_);

    // (1-9) Divide Cloud into "Ground", "Ground Above", "Ground Below", and
    // "Other"
    PIndices::Ptr ground_indices(new PIndices);
    PIndices::Ptr ground_above_indices(new PIndices);
    PIndices::Ptr ground_below_indices(new PIndices);
    PIndices::Ptr other_indices(new PIndices);
    if (!divide_by_terrain_->compute(*cloud, *initial_ground_indices,
                                     *nonground_indices, *grid_map_ptr,
                                     *ground_indices, *ground_above_indices,
                                     *ground_below_indices, *other_indices)) {
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finished: Divide By Terrain");

    // initial_ground_indices and nonground_indices are no longer needed
    initial_ground_indices->indices.clear(); // release memory
    initial_ground_indices->indices.shrink_to_fit();
    nonground_indices->indices.clear();
    nonground_indices->indices.shrink_to_fit();

    // (3) Moving Point Identification
    // input: cloud, ground_above_indices
    // output: static_indices, dynamic_indices
    PIndices::Ptr static_indices(new PIndices);
    PIndices::Ptr dynamic_indices(new PIndices);
    if (!moving_point_identification_->compute(
            loader_, cloud, ground_above_indices, *static_indices,
            *dynamic_indices)) {
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Finished: Moving Point Identification");

    // (3) Publish and Save Results
    publishCloud(cloud, ground_indices, frame_id_, pub_ground_);
    publishCloud(cloud, static_indices, frame_id_, pub_static_);
    publishCloud(cloud, dynamic_indices, frame_id_, pub_dynamic_);
    publishCloud(cloud, ground_below_indices, frame_id_, pub_ground_below_);
    publishCloud(cloud, other_indices, frame_id_, pub_other_);

    saveCloud(save_dir_ + "terrain.pcd", *terrain_pcd);
    saveCloud(save_dir_ + "static.pcd", *cloud, static_indices);
    saveCloud(save_dir_ + "dynamic.pcd", *cloud, dynamic_indices);
    saveCloud(save_dir_ + "ground_below.pcd", *cloud, ground_below_indices);
    saveCloud(save_dir_ + "ground.pcd", *cloud, ground_indices);
    saveCloud(save_dir_ + "other.pcd", *cloud, other_indices);

    RCLCPP_INFO_STREAM(this->get_logger(), "Finished: Save Cloud");
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto map_cleaner = std::make_shared<MapCleaner>();

  rclcpp::spin(map_cleaner);
  rclcpp::shutdown();

  return 0;
}
