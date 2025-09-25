#pragma once

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <map_cleaner/utils.hpp>
#include <rclcpp/rclcpp.hpp>

class DivideByTerrain {
public:
  typedef std::shared_ptr<DivideByTerrain> Ptr;
  typedef std::shared_ptr<const DivideByTerrain> ConstPtr;

private:
  std::string input_layer_name_;
  double threshold_;
  bool on_terrain_only_;

  void divide(const CloudType &cloud, const PIndices &input_indices,
              const grid_map::GridMap &grid, PIndices &ground, PIndices &above,
              PIndices &below, PIndices &other) {

    const grid_map::Matrix &terrain_layer = grid[input_layer_name_];

    // Iterate through all the input indices which are points
    for (size_t i = 0; i < input_indices.indices.size(); i++) {
      const int p_idx = input_indices.indices[i];
      const PointType &p = cloud[p_idx];

      // varible for retrieving the grid map cell index
      grid_map::Index idx;

      if (!grid.getIndex(grid_map::Position(p.x, p.y), idx)) {
        // (1) If the point is outside of the grid map, add to "other"
        other.indices.push_back(p_idx);
      } else if (!std::isfinite(terrain_layer(idx[0], idx[1]))) {
        // (2) If the terrain information is not available, add to "other"
        other.indices.push_back(p_idx);
      } else {
        // (3) If the terrain information is available, compare the height
        const float diff_z = p.z - terrain_layer(idx[0], idx[1]);
        if (diff_z < -threshold_) {
          below.indices.push_back(p_idx);
        } else if (diff_z < threshold_) {
          ground.indices.push_back(p_idx);
        } else if (diff_z <= 3.0) {
          // TODO: 3.0[m] is the maximum height of moving objects
          above.indices.push_back(p_idx);
        } else {
          other.indices.push_back(p_idx);
        }
      }
    }
  }

public:
  DivideByTerrain(const double threshold, bool on_terrain_only,
                  const std::string &input_layer = "elevation") {
    threshold_ = threshold;
    on_terrain_only_ = on_terrain_only;
    input_layer_name_ = input_layer;
  }

  bool compute(const CloudType &cloud, const PIndices &in_ground_indices,
               const PIndices &in_nonground_indices,
               const grid_map::GridMap &grid, PIndices &out_ground_indices,
               PIndices &out_above_indices, PIndices &out_below_indices,
               PIndices &out_other_indices) {
    if (!grid.exists(input_layer_name_)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("divide_by_terrain"),
                          "GridMap Does Not Have The Required Layers.");
      return false;
    }

    out_ground_indices.indices.clear();
    out_above_indices.indices.clear();
    out_below_indices.indices.clear();
    out_other_indices.indices.clear();

    if (on_terrain_only_) {
      divide(cloud, in_ground_indices, grid, out_ground_indices,
             out_above_indices, out_below_indices, out_other_indices);
      divide(cloud, in_nonground_indices, grid, out_ground_indices,
             out_above_indices, out_below_indices, out_other_indices);
    } else {
      divide(cloud, in_ground_indices, grid, out_ground_indices,
             out_above_indices, out_below_indices, out_ground_indices);
      divide(cloud, in_nonground_indices, grid, out_ground_indices,
             out_above_indices, out_below_indices, out_above_indices);
    }

    out_ground_indices.indices.shrink_to_fit();
    out_above_indices.indices.shrink_to_fit();
    out_below_indices.indices.shrink_to_fit();
    out_other_indices.indices.shrink_to_fit();

    return true;
  }
};