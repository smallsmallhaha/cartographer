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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/proto/submaps_options.pb.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_2d {

/**
 * @brief 修剪概率格网至仅含有有效区域的概率
 * 
 * @param probability_grid 
 * @return ProbabilityGrid 
 */
ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid);

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary);

/**
 * @brief 子图,含有位姿信息和概率格网
 * 
 * 主要函数:
 * InsertRangeData 用于将激光测量数据插入概率格网
 * 
 */
class Submap : public mapping::Submap {
 public:
  Submap(const MapLimits& limits, const Eigen::Vector2f& origin);
  explicit Submap(const mapping::proto::Submap2D& proto);

  void ToProto(mapping::proto::Submap* proto) const override;

  const ProbabilityGrid& probability_grid() const { return probability_grid_; }
  bool finished() const { return finished_; }

  void ToResponseProto(
      const transform::Rigid3d& global_submap_pose,
      mapping::proto::SubmapQuery::Response* response) const override;

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const RangeDataInserter& range_data_inserter);
  void Finish();

 private:
  ProbabilityGrid probability_grid_;
  bool finished_ = false;
};

// Except during initialization when only a single submap exists, there are
// always two submaps into which scans are inserted: an old submap that is used
// for matching, and a new one, which will be used for matching next, that is
// being initialized.
//
// Once a certain number of scans have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
/**
 * @brief 活跃的子图
 * 
 * 除了初始化时只有一个子图外,任何时候都有两个子图处于激活状态,
 * 旧的子图负责与scan匹配,新的用于构造下一个子图
 * 
 * 一旦一定数目的scan被插入后,新的子图就完成了初始化:
 * 此时旧的子图不再改变,新的子图现在成了旧的子图,用于scan-to-map匹配.
 * 
 */
class ActiveSubmaps {
 public:
  explicit ActiveSubmaps(const proto::SubmapsOptions& options);

  ActiveSubmaps(const ActiveSubmaps&) = delete;
  ActiveSubmaps& operator=(const ActiveSubmaps&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  /**
   * @brief 正在匹配(构造)的子图的编号
   * 
   * @return int matching_index 
   */
  int matching_index() const;

  // Inserts 'range_data' into the Submap collection.
  /**
   * @brief 向子图集submaps_中插入距离测量数据
   * 
   * @param range_data 
   */
  void InsertRangeData(const sensor::RangeData& range_data);

  std::vector<std::shared_ptr<Submap>> submaps() const;

 private:
  /**
   * @brief 完成并销毁第一个子图,matching_submap_index_加一
   * 
   */
  void FinishSubmap();
  /**
   * @brief 添加一个子图,若子图有两个,则销毁第一个子图再添加新的
   * 
   * @param origin 
   */
  void AddSubmap(const Eigen::Vector2f& origin);

  const proto::SubmapsOptions options_;
  // 正在匹配的子图的ID,用已经完成匹配的子图的个数表示(子图销毁才算完成)
  int matching_submap_index_ = 0;
  std::vector<std::shared_ptr<Submap>> submaps_;
  RangeDataInserter range_data_inserter_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
