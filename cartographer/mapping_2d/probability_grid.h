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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/proto/probability_grid.pb.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

// Represents a 2D grid of probabilities.
/**
 * @brief 2D概率格网
 * 
 */
class ProbabilityGrid {
 public:
  explicit ProbabilityGrid(const MapLimits& limits)
      : limits_(limits),
        cells_(limits_.cell_limits().num_x_cells *
                   limits_.cell_limits().num_y_cells,
               mapping::kUnknownProbabilityValue) {}

  explicit ProbabilityGrid(const proto::ProbabilityGrid& proto)
      : limits_(proto.limits()), cells_() {
    if (proto.has_min_x()) {
      known_cells_box_ =
          Eigen::AlignedBox2i(Eigen::Vector2i(proto.min_x(), proto.min_y()),
                              Eigen::Vector2i(proto.max_x(), proto.max_y()));
    }
    cells_.reserve(proto.cells_size());
    for (const auto cell : proto.cells()) {
      CHECK_LE(cell, std::numeric_limits<uint16>::max());
      cells_.push_back(cell);
    }
  }

  // Returns the limits of this ProbabilityGrid.
  /**
   * @brief 返回概率格网边界
   * 
   * @return const MapLimits& 
   */
  const MapLimits& limits() const { return limits_; }

  // Finishes the update sequence.
  /**
   * @brief 停止更新,主要是删除该点的kUpdataMarker标记
   * 
   */
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(cells_[update_indices_.back()], mapping::kUpdateMarker);
      cells_[update_indices_.back()] -= mapping::kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  /**
   * @brief 设置未知格网点的概率值,更新已知概率格网中的已知区域边界
   * 
   * !!! 注意: 每一个格网点只能调用一次该函数
   * 
   * @param cell_index 
   * @param probability 
   */
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability) {
    uint16& cell = cells_[ToFlatIndex(cell_index)];
    CHECK_EQ(cell, mapping::kUnknownProbabilityValue);
    cell = mapping::ProbabilityToValue(probability);
    known_cells_box_.extend(cell_index.matrix());
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  /**
   * @brief 应用查找表,更新格网点的概率值
   * 
   * 公式: M_new = OddsInv( odds * Odds( M_old ) )
   * 详情请看论文
   * 
   * @param cell_index 
   * @param table 
   * @return true 
   * @return false 
   */
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), mapping::kUpdateMarker);
    const int flat_index = ToFlatIndex(cell_index);
    // 获取格网点概率值(uint16)
    uint16& cell = cells_[flat_index];
    // 若已经设定完概率值,则直接返回false
    if (cell >= mapping::kUpdateMarker) {
      return false;
    }
    // 若还未设定概率值,需要对格网点概率值进行更新,并且添加到update_indices_队列
    // 这样做是为了 当你直接调用ApplyLookupTable多次对一个点进行更新时,保证只有第一个调用是有效的
    // 要想多次更新,需要调用FinishUpdate"解开"对已更新单元格的锁
    update_indices_.push_back(flat_index);
    cell = table[cell];
    DCHECK_GE(cell, mapping::kUpdateMarker);
    known_cells_box_.extend(cell_index.matrix());
    return true;
  }

  // Returns the probability of the cell with 'cell_index'.
  /**
   * @brief 获取格网点概率值,从 [0-2^16-1]>>1 到 [0.1,0.9] 的映射
   * 
   * @param cell_index 
   * @return float 
   */
  float GetProbability(const Eigen::Array2i& cell_index) const {
    if (limits_.Contains(cell_index)) {
      return mapping::ValueToProbability(cells_[ToFlatIndex(cell_index)]);
    }
    return mapping::kMinProbability;
  }

  // Returns true if the probability at the specified index is known.
  /**
   * @brief 格网点概率是否已知
   * 
   * @param cell_index 
   * @return true 
   * @return false 
   */
  bool IsKnown(const Eigen::Array2i& cell_index) const {
    return limits_.Contains(cell_index) &&
           cells_[ToFlatIndex(cell_index)] != mapping::kUnknownProbabilityValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  /**
   * @brief 计算已知格网点的边界
   * 
   * @param offset 
   * @param limits 
   */
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const {
    if (known_cells_box_.isEmpty()) {
      *offset = Eigen::Array2i::Zero();
      *limits = CellLimits(1, 1);
    } else {
      *offset = known_cells_box_.min().array();
      *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                           known_cells_box_.sizes().y() + 1);
    }
  }

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  /**
   * @brief 扩大边界,直到概率格网内包含点point
   * 
   * 必须在FinishUpdate()立即调用,在调用ApplyLookupTable之前调用
   * 
   * @param point 
   */
  void GrowLimits(const Eigen::Vector2f& point) {
    CHECK(update_indices_.empty());
    while (!limits_.Contains(limits_.GetCellIndex(point))) {
      // 每次边界的角点增加resolution * num_cells / 2,总面积扩展到之前的四倍
      const int x_offset = limits_.cell_limits().num_x_cells / 2;
      const int y_offset = limits_.cell_limits().num_y_cells / 2;
      const MapLimits new_limits(
          limits_.resolution(),
          limits_.max() +
              limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
          CellLimits(2 * limits_.cell_limits().num_x_cells,
                     2 * limits_.cell_limits().num_y_cells));
      const int stride = new_limits.cell_limits().num_x_cells;
      const int offset = x_offset + stride * y_offset;
      const int new_size = new_limits.cell_limits().num_x_cells *
                           new_limits.cell_limits().num_y_cells;
      std::vector<uint16> new_cells(new_size,
                                    mapping::kUnknownProbabilityValue);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
              cells_[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      cells_ = new_cells;
      limits_ = new_limits;
      if (!known_cells_box_.isEmpty()) {
        known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
      }
    }
  }

  proto::ProbabilityGrid ToProto() const {
    proto::ProbabilityGrid result;
    *result.mutable_limits() = cartographer::mapping_2d::ToProto(limits_);
    result.mutable_cells()->Reserve(cells_.size());
    for (const auto cell : cells_) {
      result.mutable_cells()->Add(cell);
    }
    CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                      "not supported. Finish the update first.";
    if (!known_cells_box_.isEmpty()) {
      result.set_max_x(known_cells_box_.max().x());
      result.set_max_y(known_cells_box_.max().y());
      result.set_min_x(known_cells_box_.min().x());
      result.set_min_y(known_cells_box_.min().y());
    }
    return result;
  }

 private:
  // Converts a 'cell_index' into an index into 'cells_'.
  // 注意: cells_为列优先数组
  int ToFlatIndex(const Eigen::Array2i& cell_index) const {
    CHECK(limits_.Contains(cell_index)) << cell_index;
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }

  MapLimits limits_;
  std::vector<uint16> cells_;  // Highest bit is update marker.
  std::vector<int> update_indices_;

  // Bounding box of known cells to efficiently compute cropping limits.
  Eigen::AlignedBox2i known_cells_box_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
