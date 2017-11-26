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

#include "cartographer/mapping_2d/ray_casting.h"

namespace cartographer {
namespace mapping_2d {

namespace {

// Factor for subpixel accuracy of start and end point.
/**
 * @brief 激光开始点和结束点的精度因子
 * 
 */
constexpr int kSubpixelScale = 1000;

// We divide each pixel in kSubpixelScale x kSubpixelScale subpixels. 'begin'
// and 'end' are coordinates at subpixel precision. We compute all pixels in
// which some part of the line segment connecting 'begin' and 'end' lies.
void CastRay(const Eigen::Array2i& begin, const Eigen::Array2i& end,
             const std::vector<uint16>& miss_table,
             ProbabilityGrid* const probability_grid) {
  // For simplicity, we order 'begin' and 'end' by their x coordinate.
  // 为了简单一些,我们使得begin.x()<end.x()
  if (begin.x() > end.x()) {
    CastRay(end, begin, miss_table, probability_grid);
    return;
  }

  CHECK_GE(begin.x(), 0);
  CHECK_GE(begin.y(), 0);
  CHECK_GE(end.y(), 0);

  // Special case: We have to draw a vertical line in full pixels, as 'begin'
  // and 'end' have the same full pixel x coordinate.
  /**
   * @brief 特例,激光与y轴垂直,用miss_table更新线段上所有的概率信息
   * 
   */
  if (begin.x() / kSubpixelScale == end.x() / kSubpixelScale) {
    Eigen::Array2i current(begin.x() / kSubpixelScale,
                           std::min(begin.y(), end.y()) / kSubpixelScale);
    const int end_y = std::max(begin.y(), end.y()) / kSubpixelScale;
    for (; current.y() <= end_y; ++current.y()) {
      probability_grid->ApplyLookupTable(current, miss_table);
    }
    return;
  }

  const int64 dx = end.x() - begin.x();
  const int64 dy = end.y() - begin.y();
  const int64 denominator = 2 * kSubpixelScale * dx;

  // The current full pixel coordinates. We begin at 'begin'.
  Eigen::Array2i current = begin / kSubpixelScale;

  // To represent subpixel centers, we use a factor of 2 * 'kSubpixelScale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * kSubpixelScale) / (2 * kSubpixelScale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * kSubpixelScale)
  // | | | | -- center of first subpixel = 1 / (2 * kSubpixelScale)
  // +-+-+-+ -- 0 = 0 / (2 * kSubpixelScale)

  // The center of the subpixel part of 'begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  int64 sub_y = (2 * (begin.y() % kSubpixelScale) + 1) * dx;

  // The distance from the from 'begin' to the right pixel border, to be divided
  // by 2 * 'kSubpixelScale'.
  const int first_pixel =
      2 * kSubpixelScale - 2 * (begin.x() % kSubpixelScale) - 1;
  // The same from the left pixel border to 'end'.
  const int last_pixel = 2 * (end.x() % kSubpixelScale) + 1;

  // The full pixel x coordinate of 'end'.
  const int end_x = std::max(begin.x(), end.x()) / kSubpixelScale;

  // Move from 'begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      probability_grid->ApplyLookupTable(current, miss_table);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        probability_grid->ApplyLookupTable(current, miss_table);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      sub_y += dy * 2 * kSubpixelScale;
    }
    // Move from the pixel border on the right to 'end'.
    sub_y += dy * last_pixel;
    probability_grid->ApplyLookupTable(current, miss_table);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      probability_grid->ApplyLookupTable(current, miss_table);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), end.y() / kSubpixelScale);
    return;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    probability_grid->ApplyLookupTable(current, miss_table);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      probability_grid->ApplyLookupTable(current, miss_table);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * kSubpixelScale;
  }
  sub_y += dy * last_pixel;
  probability_grid->ApplyLookupTable(current, miss_table);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    probability_grid->ApplyLookupTable(current, miss_table);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), end.y() / kSubpixelScale);
}

/**
 * @brief 扩展概率格网范围,直到概率格网包含所有激光观测观测数据为止
 * 
 * @param range_data 
 * @param probability_grid 
 */
void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  // 包含激光测量中心
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  constexpr float kPadding = 1e-6f;
  // 包含hits和misses
  for (const Eigen::Vector3f& hit : range_data.returns) {
    bounding_box.extend(hit.head<2>());
  }
  for (const Eigen::Vector3f& miss : range_data.misses) {
    bounding_box.extend(miss.head<2>());
  }
  // 扩展概率格网范围
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

}  // namespace

// CastRays实现
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space,
              ProbabilityGrid* const probability_grid) {
  // 1. 扩展概率格网范围
  GrowAsNeeded(range_data, probability_grid);

  // 2. 获得超分辨率下的地图范围 单元格数量变为原来的kSubpixelScale*kSubpixelScale倍
  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  // 获得原点坐标
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const Eigen::Vector3f& hit : range_data.returns) {
    // 获得hit点在超分辨率坐标下的坐标
    ends.push_back(superscaled_limits.GetCellIndex(hit.head<2>()));
    // 3. 用hit_table更新该坐标的概率
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  if (!insert_free_space) {
    // 如果设置insert_free_space为false,则不处理range_data.misses数据,直接返回
    // 概率格网将没有misses的数据
    return;
  }

  // Now add the misses.
  // 4-1. 用miss_table更新orgin到ends线段上所有点的概率
  for (const Eigen::Array2i& end : ends) {
    CastRay(begin, end, miss_table, probability_grid);
  }

  // Finally, compute and add empty rays based on misses in the scan.
  // 4-2. 用miss_table更新orgin到misses线段上所有点的概率
  for (const Eigen::Vector3f& missing_echo : range_data.misses) {
    CastRay(begin, superscaled_limits.GetCellIndex(missing_echo.head<2>()),
            miss_table, probability_grid);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
