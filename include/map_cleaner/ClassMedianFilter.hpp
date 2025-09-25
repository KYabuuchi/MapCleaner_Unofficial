#pragma once
#include "map_cleaner/utils.hpp"

class ClassMedianFilter {
public:
  explicit ClassMedianFilter(const int neightbor_size);

  void filter(const CloudType::Ptr &cloud,
              const PIndices::ConstPtr &static_indices,
              const PIndices::ConstPtr &dynamic_indices,
              PIndices &out_static_indices, PIndices &out_dynamic_indices);

private:
  int neighbor_size_;
};