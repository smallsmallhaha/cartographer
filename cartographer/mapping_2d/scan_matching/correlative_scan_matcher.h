/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

typedef std::vector<Eigen::Array2i> DiscreteScan;

// Describes the search space.
/**
 * @brief 描述搜索空间
 */
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  /**
   * @brief 线性搜索窗口
   */
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  // 注意, 这里的线性搜索窗口和角度搜索窗口只是半径
  SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::PointCloud& point_cloud, double resolution);

  // For testing.
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // Tightens the search window as much as possible.
  /**
   * @brief 尽可能的缩小搜索窗口
   */
  void ShrinkToFit(const std::vector<DiscreteScan>& scans,
                   const CellLimits& cell_limits);

  // 角度步长个数
  int num_angular_perturbations;
  /**
   * @brief 角度摄动步长
   * 
   * 典型值： 当最大距离为 10m， 分辨率为 5cm 时， 摄动步长为 0.2865°
   */
  double angular_perturbation_step_size;
  // 分辨率， 即线性摄动步长
  double resolution;
  // 点云的个数, 2 * num_angular_perturbations + 1, 每个旋转角度对应一个新的点云
  int num_scans;
  // 线性搜索窗口, 每一个旋转角对应一个
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

// Generates a collection of rotated scans.
/**
 * @brief 生成旋转 scan 的集合
 * 
 * 将 scan 在角度搜索窗口 [-angular_search_window, angular_search_window] 内
 * 依次按照一定步长旋转某些角度， 返回所有旋转后的点云
 */
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
/**
 * @brief 离散化旋转点云
 * 
 * 将所有点云变换后离散化到格网上
 */
std::vector<DiscreteScan> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
/**
 * @brief 候选解
 * 
 * 结构: scan在旋转集的id, x偏移量, y偏移量, 匹配分数
 */
struct Candidate {
  Candidate(const int init_scan_index, const int init_x_index_offset,
            const int init_y_index_offset,
            const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate& other) const { return score < other.score; }
  bool operator>(const Candidate& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
