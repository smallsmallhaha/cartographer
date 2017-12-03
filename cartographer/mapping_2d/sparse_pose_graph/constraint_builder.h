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

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
transform::Rigid2d ComputeSubmapPose(const Submap& submap);

// Asynchronously computes constraints.
//
// Intermingle an arbitrary number of calls to MaybeAddConstraint() or
// MaybeAddGlobalConstraint, then call WhenDone(). After all computations are
// done the 'callback' will be called with the result and another
// MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
//
// This class is thread-safe.
/** 
 * @brief 计算约束(异步)
 * 
 * 以下是一个计算周期(后面的注释里会写成'周期')
 * 1. 任意多次调用MaybeAdd(Global)Constraint(), 最后调用一次 WhenDone() 注册回调函数
 * 2. 上述两种调用分别对应约束的计算和优化的计算, 顺序为; 先完成所有约束计算, 然后调用回调函数
 *    完成优化计算
 * 上述'周期'完成了就是本系统所做的大部分计算,重复上述过程即可
 * 
 * 线程安全
 * 
 */
class ConstraintBuilder {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint;
  using Result = std::vector<Constraint>;

  ConstraintBuilder(
      const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions&
          options,
      common::ThreadPool* thread_pool);
  ~ConstraintBuilder();

  ConstraintBuilder(const ConstraintBuilder&) = delete;
  ConstraintBuilder& operator=(const ConstraintBuilder&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
  // 'initial_relative_pose' is relative to the 'submap'.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  // 添加约束,调度计算约束
  void MaybeAddConstraint(
      const mapping::SubmapId& submap_id, const Submap* submap,
      const mapping::NodeId& node_id,
      const mapping::TrajectoryNode::Data* const constant_data,
      const transform::Rigid2d& initial_relative_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  // 添加全局约束,调度计算全局约束
  void MaybeAddGlobalConstraint(
      const mapping::SubmapId& submap_id, const Submap* submap,
      const mapping::NodeId& node_id,
      const mapping::TrajectoryNode::Data* const constant_data);

  // Must be called after all computations related to one node have been added.
  /**
   * @brief  scan 数据添加结束
   * 
   * 必须在与一个 node 相关的所有计算任务都被添加完毕之后(立即)调用
   */
  void NotifyEndOfScan();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by MaybeAddConstraint() have finished.
  // 注册回调函数,当所有的MaybeAdd(Global)Constraint()触发的计算完毕后调用回调函数
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished scans.
  int GetNumFinishedScans();

  // Delete data related to 'submap_id'.
  // 删除submap_id对应的scan matcher
  void DeleteScanMatcher(const mapping::SubmapId& submap_id);

 private:
  struct SubmapScanMatcher {
    const ProbabilityGrid* probability_grid;
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher>
        fast_correlative_scan_matcher;
  };

  // Either schedules the 'work_item', or if needed, schedules the scan matcher
  // construction and queues the 'work_item'.
  // 若submap_id有对应的scan-matcher.则调度work_item,否则构造scan-matcher并调度work-item
  void ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      const mapping::SubmapId& submap_id, const ProbabilityGrid* submap,
      const std::function<void()>& work_item) REQUIRES(mutex_);

  // Constructs the scan matcher for a 'submap', then schedules its work items.
  // 构造submap_id对应的scan-matcher并调度work-item
  void ConstructSubmapScanMatcher(const mapping::SubmapId& submap_id,
                                  const ProbabilityGrid* submap)
      EXCLUDES(mutex_);

  // Returns the scan matcher for a submap, which has to exist.
  // 返回submap_id对应的scan matcher,返回值必须非空
  const SubmapScanMatcher* GetSubmapScanMatcher(
      const mapping::SubmapId& submap_id) EXCLUDES(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'compressed_point_cloud' do not change
  // anymore. As output, it may create a new Constraint in 'constraint'.
  // 在工作线程中计算并添加约束
  // 计算过程中假设submap和data对应的数据都不会改变
  void ComputeConstraint(
      const mapping::SubmapId& submap_id, const Submap* submap,
      const mapping::NodeId& node_id, bool match_full_submap,
      const mapping::TrajectoryNode::Data* const constant_data,
      const transform::Rigid2d& initial_relative_pose,
      std::unique_ptr<Constraint>* constraint) EXCLUDES(mutex_);

  // Decrements the 'pending_computations_' count. If all computations are done,
  // runs the 'when_done_' callback and resets the state.
  // 降低pending_computations_计数
  // 如果所有计算完毕,运行when_done_回调函数
  /**
   * @brief 完成一个计算任务
   * 
   * 在两处被调用:
   * 1. 每计算完一个约束被调用, 对应的 pending_computations_ 计数减一
   * 2. 每完成一个闭环计算周期, 对应的 pending_computations_ 减一
   *    注意: 1和2是并行的, 本函数的精巧设计可以使得某个闭环周期的所有约束被计算完毕后才开始
   *         优化, 具体为:
   *         当对应的 pending_computations_ 计数减为0时, 执行回调函数 when_done_
   */
  void FinishComputation(int computation_index) EXCLUDES(mutex_);

  
 private:
  const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions options_;
  common::ThreadPool* thread_pool_;
  common::Mutex mutex_;

  // 'callback' set by WhenDone().
  /**
   * @brief 回调函数, 通过 WhenDone() 设置, 在每一个周期的调用最后被调用, 用于闭环优化
   */
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // Index of the scan in reaction to which computations are currently
  // added. This is always the highest scan index seen so far, even when older
  // scans are matched against a new submap.
  /**
   * @brief 当前添加的 scan 的 id
   * 
   * 总是最大的 scan id
   */
  int current_computation_ GUARDED_BY(mutex_) = 0;

  // For each added scan, maps to the number of pending computations that were
  // added for it.
  /** @brief scan id -> computation_count
   * 
   * computation_count 为 0 的项将会被销毁
   * 
   */
  std::map<int, int> pending_computations_ GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries.
  // 当前正在被计算的约束
  std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of already constructed scan matchers by 'submap_id'.
  // submap_id和对应的scan matcher
  std::map<mapping::SubmapId, SubmapScanMatcher> submap_scan_matchers_
      GUARDED_BY(mutex_);

  // Map by 'submap_id' of scan matchers under construction, and the work
  // to do once construction is done.
  // 一旦submap_id对应的scanmatcher构造完毕,就开始调度相应的作业
  std::map<mapping::SubmapId, std::vector<std::function<void()>>>
      submap_queued_work_items_ GUARDED_BY(mutex_);

  common::FixedRatioSampler sampler_;
  scan_matching::CeresScanMatcher ceres_scan_matcher_;

  // Histogram of scan matcher scores.
  common::Histogram score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
