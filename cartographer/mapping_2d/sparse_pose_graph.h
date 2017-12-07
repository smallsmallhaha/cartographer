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

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/trajectory_connectivity_state.h"
#include "cartographer/mapping_2d/sparse_pose_graph/constraint_builder.h"
#include "cartographer/mapping_2d/sparse_pose_graph/optimization_problem.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_2d {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each scan has been matched against one or more submaps (adding a constraint
// for each match), both poses of scans and of submaps are to be optimized.
// All constraints are between a submap i and a scan j.
/**
 * 
 * @brief SPA (Sparse Pose Adjustment) 闭环优化算法实现
 * 
 * 原理: 请参照论文 Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
 * 
 * 简单的来说,就是根据 scan 和 submap 的约束建立稀疏位姿图,进行优化
 * 
 * 使用方法:
 * 
 * 1. 初始化
 * 2. 计算约束: 使用 AddScan/AddImuData/AddOdometerData/AddFixedFramePoseData 添加数据,
 *    同时在工作线程中计算 node 和 submap 的约束
 * 3. 优化: 调用 RunFinalOptimization() 等待所有约束计算完成,完成最终的优化计算
 * 
 * 请注意: 第3步在数据采集完毕后,按Ctrl+C退出后才开始,optimization_problem_.Solve()也是最后
 *        才执行的
 * 
 * 
 * 实现细节简述
 * 最重要的几个成员和函数:
 * public:
 * AddScan()                    外部接口, 添加数据, 计算约束和优化
 * RunFinalOptimization()       程序关闭前最后的优化
 * private:
 * work_queue_                  作业队列, 控制工作流程
 * run_loop_closure_            辅助工作队列控制闭环优化
 * optimization_problem_        闭环优化
 * constraint_builder_          后台计算约束和执行闭环优化(可选)
 * constraints_                 约束, 储存约束信息
 * submap_data_                 存储子图
 * num_trajectory_nodes_        存储节点
 * AddWorkItem()                添加或执行作业项
 * ComputeConstraintsForScan()  为节点计算约束
 * ComputeConstraint()          为节点和子图计算约束
 * HandleWorkQueue()            处理作业队列(为指定的优化设置回调函数)
 * 
 */
class SparsePoseGraph : public mapping::SparsePoseGraph {
 public:
  SparsePoseGraph(const mapping::proto::SparsePoseGraphOptions& options,
                  common::ThreadPool* thread_pool);
  ~SparsePoseGraph() override;

  SparsePoseGraph(const SparsePoseGraph&) = delete;
  SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

  // Adds a new node with 'constant_data' and a 'pose' that will later be
  // optimized. The 'pose' was determined by scan matching against
  // 'insertion_submaps.front()' and the scan was inserted into the
  // 'insertion_submaps'. If 'insertion_submaps.front().finished()' is
  // 'true', this submap was inserted into for the last time.
  /**
   * @brief
   * 
   * 1. 添加新 node
   * 2. 若 insertion_submaps.back() 第一次出现, 则 submap_data_ 添加新项
   * 3. 调用 ComputeConstraintsForScan() 为 node 计算约束
   */
  void AddScan(
      std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      EXCLUDES(mutex_);

  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data);
  void AddOdometerData(int trajectory_id,
                       const sensor::OdometryData& odometry_data);
  void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data);

  void FreezeTrajectory(int trajectory_id) override;
  void AddSubmapFromProto(int trajectory_id,
                          const transform::Rigid3d& initial_pose,
                          const mapping::proto::Submap& submap) override;
  void AddNodeFromProto(int trajectory_id, const transform::Rigid3d& pose,
                        const mapping::proto::Node& node) override;
  void AddTrimmer(std::unique_ptr<mapping::PoseGraphTrimmer> trimmer) override;
  void RunFinalOptimization() override;
  std::vector<std::vector<int>> GetConnectedTrajectories() override;
  int num_submaps(int trajectory_id) EXCLUDES(mutex_) override;
  mapping::SparsePoseGraph::SubmapData GetSubmapData(
      const mapping::SubmapId& submap_id) EXCLUDES(mutex_) override;
  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
  GetAllSubmapData() EXCLUDES(mutex_) override;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id)
      EXCLUDES(mutex_) override;
  std::vector<std::vector<mapping::TrajectoryNode>> GetTrajectoryNodes()
      override EXCLUDES(mutex_);
  std::vector<Constraint> constraints() override EXCLUDES(mutex_);

 private:
  // The current state of the submap in the background threads. When this
  // transitions to kFinished, all scans are tried to match against this submap.
  // Likewise, all new scans are matched against submaps which are finished.
  // 
  // 
  /**
   * @brief 子图状态
   * 
   * 后台线程中子图的当前状态. 当状态转换为kFinished时, 所有的scan都试图与该submap匹配
   * 同样的, 所有的新的scan都会与kFinished状态的submap匹配
   */
  enum class SubmapState { kActive, kFinished, kTrimmed };
  /**
   * @brief 子图数据
   * 
   * 结构: 子图 submap  与子图有约束的节点集 node_ids  子图状态 state
   */
  struct SubmapData {
    std::shared_ptr<const Submap> submap;

    // IDs of the scans that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    std::set<mapping::NodeId> node_ids;

    SubmapState state = SubmapState::kActive;
  };

  // Handles a new work item.
  /**
   * @brief 添加一个新的作业项
   * 
   * @param work_item 作业项
   * 
   * 若作业队列 work_queue_ 为空,则立即执行,否则加入作业队列,等待执行
   */
  void AddWorkItem(const std::function<void()>& work_item) REQUIRES(mutex_);

  // Adds connectivity and sampler for a trajectory if it does not exist.
  void AddTrajectoryIfNeeded(int trajectory_id) REQUIRES(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  /**
   * @brief 给 optimization_problem_ 添加子图数据
   * 
   * @param trajectory_id
   * @param insertion_submaps
   *
   * @return 
   * 
   * 给 optimization_problem_ 添加子图数据,为 insertion_submaps 的每个元素提供一个入口
   */
  std::vector<mapping::SubmapId> GrowSubmapTransformsAsNeeded(
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      REQUIRES(mutex_);

  // Adds constraints for a scan, and starts scan matching in the background.
  /**
   * @brief 为 node 和 submap 添加和计算约束
   * 
   * @param node_id  node 对应的 node_id
   * @param insertion_submaps  插入子图(个数一般为2)
   * @param newly_finished_submap  旧子图是否刚刚完成(即新子图构造完成)
   * 
   * 步骤:
   * 1. 为 optimization_problem_ 添加 node 数据
   * 2. 将 node 和 insertion_submaps 的约束加入 constraints_
   *      (注: constraint类型为Constraint::INTRA_SUBMAP)
   * 3. 计算指定 node 和所有已完成 submap 的约束
   * 4. 若旧子图刚刚完成(newly_finished_submap为true),则计算旧子图和
   *    optimization_problem_ 中所有 node 的约束
   *      (注: 2和3计算完毕后的constraint结果将被加入constraints_, constraint类型
   *      为Constraint::INTER_SUBMAP)
   * 5. 若本函数执行次数达到一定值,则启动闭环优化,过程为:
   *      若工作队列 work_queue_ 非空,则通过调用 HandleWorkQueue() 使用
   *        ConstraintBuilder::WhenDone() 注册回调函数;
   *      否则说明有约束没算完,算完之后调用上次注册的回调函数
   * 
   */
  void ComputeConstraintsForScan(
      const mapping::NodeId& node_id,
      std::vector<std::shared_ptr<const Submap>> insertion_submaps,
      bool newly_finished_submap) REQUIRES(mutex_);

  // Computes constraints for a scan and submap pair.
  /**
   * @brief 为 node 和 submap 计算约束
   * 
   * 若 node 和 submap 属于同一个 trajectory, 或者 node 和 submap 所属的不同
   * trajctory 之间不久前刚计算过约束,
   *   则使用 constraint_builder_.MaybeAddConstraint()
   *   否则用 constraint_builder_.MaybeAddGlobalConstraint()
   */
  void ComputeConstraint(const mapping::NodeId& node_id,
                         const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  // Adds constraints for older scans whenever a new submap is finished.
  /**
   * @brief 为新完成的 submap 和已有的所有 node 计算约束
   * 
   */
  void ComputeConstraintsForOldScans(const mapping::SubmapId& submap_id)
      REQUIRES(mutex_);

  // Registers the callback to run the optimization once all constraints have
  // been computed, that will also do all work that queue up in 'work_queue_'.
  /**
   * @brief 处理作业队列
   * 
   * 只在 ComputeConstraintsForScan() 中被调用, 作用是注册 constraint_builder_ 的
   * 回调函数, 回调函数内容如下:
   * 1. 将计算完毕的约束结果加入 constraints_
   * 2. 调用 RunOptimization() 运行优化(优化计算在后台做)
   * 3. 修剪 SparsePoseGraph
   * 4. 若 run_loop_closure_ 为 false, 则循环执行 work_queue_ 中的作业项; 若执行过程中
   *    run_loop_closure_ 突然变为 true, 说明本次优化的太慢了, 以至于下一轮优化已经开始
   */
  void HandleWorkQueue() REQUIRES(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() EXCLUDES(mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() EXCLUDES(mutex_);

  // Computes the local to global frame transform based on the given optimized
  // 'submap_transforms'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const mapping::MapById<mapping::SubmapId, sparse_pose_graph::SubmapData>&
          submap_transforms,
      int trajectory_id) const REQUIRES(mutex_);

  mapping::SparsePoseGraph::SubmapData GetSubmapDataUnderLock(
      const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  common::Time GetLatestScanTime(const mapping::NodeId& node_id,
                                 const mapping::SubmapId& submap_id) const
      REQUIRES(mutex_);

  // Updates the trajectory connectivity structure with the new constraints.
  void UpdateTrajectoryConnectivity(
      const sparse_pose_graph::ConstraintBuilder::Result& result)
      REQUIRES(mutex_);

 private:
  const mapping::proto::SparsePoseGraphOptions options_;
  common::Mutex mutex_;

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  /**
   * @brief 作业队列
   * 
   * 作业队列,若非空,则作业项将被加入作业队列,后续执行; 否则直接执行
   * 
   * 可能存在的成员:
   * ComputeConstraintsForScan()
   * optimization_problem_.AddImuData()
   * optimization_problem_.AddOdometerData()
   * frozen_trajectories_.insert()                  in ::FreezeTrajectory()
   * optimization_problem_.AddSubmap()              in ::AddSubmapFromProto()
   * optimization_problem_.AddTrajectoryNode()      in ::AddNodeFromProto()
   * trimmers_.emplace_back()                       in ::AddTrimmer()
   */
  std::unique_ptr<std::deque<std::function<void()>>> work_queue_
      GUARDED_BY(mutex_);

  // How our various trajectories are related.
  mapping::TrajectoryConnectivityState trajectory_connectivity_state_;

  // We globally localize a fraction of the scans from each trajectory.
  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ GUARDED_BY(mutex_);

  // Number of scans added since last loop closure.
  int num_scans_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

  // Whether the optimization has to be run before more data is added.
  bool run_loop_closure_ GUARDED_BY(mutex_) = false;

  // Current optimization problem.
  // 当前的OptimizationProblem
  sparse_pose_graph::OptimizationProblem optimization_problem_;
  // 当前的ConstraintBuilder, 用于计算约束
  sparse_pose_graph::ConstraintBuilder constraint_builder_ GUARDED_BY(mutex_);
  // 保存 ConstraintBuilder 计算得到的约束, 供 optimization_problem_.Solve(constraints_,
  //   frozen_trajectories_) 进行最后的优化
  std::vector<Constraint> constraints_ GUARDED_BY(mutex_);

  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  mapping::NestedVectorsById<SubmapData, mapping::SubmapId> submap_data_
      GUARDED_BY(mutex_);

  // Data that are currently being shown.
  mapping::NestedVectorsById<mapping::TrajectoryNode, mapping::NodeId>
      trajectory_nodes_ GUARDED_BY(mutex_);
  int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

  // Current submap transforms used for displaying data.
  mapping::MapById<mapping::SubmapId, sparse_pose_graph::SubmapData>
      optimized_submap_transforms_ GUARDED_BY(mutex_);

  // List of all trimmers to consult when optimizations finish.
  std::vector<std::unique_ptr<mapping::PoseGraphTrimmer>> trimmers_
      GUARDED_BY(mutex_);

  // Set of all frozen trajectories not being optimized.
  std::set<int> frozen_trajectories_ GUARDED_BY(mutex_);

  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  class TrimmingHandle : public mapping::Trimmable {
   public:
    TrimmingHandle(SparsePoseGraph* parent);
    ~TrimmingHandle() override {}

    int num_submaps(int trajectory_id) const override;
    void MarkSubmapAsTrimmed(const mapping::SubmapId& submap_id) override;

   private:
    SparsePoseGraph* const parent_;
  };
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_
